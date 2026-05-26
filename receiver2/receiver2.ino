// Must add #include <TFT_eSPI_setup.h> line to TFT_eSPI library user setup file

// To upload: hold down BOOT on right, briefly press EN on left, release BOOT
// EN is reset, may need to press this to start code running

// ****** IMPORTANT: must get JTAG port to be recognised by Windows, might take some fiddling of connector

// Refer to https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html

// use the board esp32 -> ESP32 Dev Module if using WROOM module
// 192.168.0.99 (box with display and large buttons)

/*
 * Pixel Update Receiver for ESP32 T-Display (ST7735 128x160)
 * Receives per-pixel updates (x, y, RGB565) over TCP and applies them.
 * Protocol v2 (little-endian):
 *   Header: 'P' 'X' 'U' 'P' (4 bytes) + version (1 byte, 0x02) + frame_id (uint32 LE) + count (uint16)
 *   Body:   count entries of: x (uint16), y (uint16), color (uint16 LE)
 *
 * Optimized for high frame rates with:
 * - Fast SPI clock (80MHz default, configurable)
 * - DMA support for efficient display updates
 * - Run-length encoding support for reduced bandwidth
 */


#include "SPI.h"
#include "TFT_eSPI.h"
#include <ezButton.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <esp_heap_caps.h>  // for PSRAM allocations
#include <PNGdec.h>         // for PNG decode
#include "credentials.h"

#define TFT_MADCTL 0x36
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08

TFT_eSPI tft = TFT_eSPI();

// Display dimensions
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 160
#define FRAMEBUFFER_SIZE DISPLAY_WIDTH * DISPLAY_HEIGHT

#define CLIENT_SEND 0x31
//#define CLIENT_SEND_FULL 0x32
#define CLIENT_WAIT 0x33
#define TIMEOUT 10000
uint8_t CLIENT_SEND_FULL[] = {
  0x32,                       // magic code
  0x01,                       // version number
  DISPLAY_WIDTH >> 8, 
  DISPLAY_WIDTH & 0xFF,
  DISPLAY_HEIGHT >> 8,
  DISPLAY_HEIGHT & 0xFF};

// Try the fastest stable SPI clock for the panel; lower to 40000000 if unstable
const uint32_t SPI_TARGET_FREQ = 80000000;

// WiFi credentials - UPDATE THESE WITH YOUR NETWORK
//const char* ssid = "";
//const char* password = "";

// Network settings
WiFiServer server(8090);  // dedicated port for pixel updates
#define MAX_CLIENTS 5
WiFiClient clients[MAX_CLIENTS];
int activeClient = -1;

// PNG decode
PNG png;
#define PNG_NO_ALPHA 0xffffffff

#define BLUE_BUTTON   22
#define YELLOW_BUTTON 21
#define RED_BUTTON    16
#define GREEN_BUTTON  17

// Protocol constants (v2 adds frame_id to the header)
const uint8_t MAGIC[4] = {'P', 'X', 'U', 'P'};
const uint8_t PROTO_VERSION = 0x02;
const size_t HEADER_SIZE = 11;  // MAGIC (4) + version (1) + frame_id (4) + count (2)
const uint8_t MAGIC_RUN[4] = {'P', 'X', 'U', 'R'};
const uint8_t RUN_VERSION = 0x01;
const size_t RUN_HEADER_SIZE = 11;  // MAGIC_RUN (4) + version (1) + frame_id (4) + count (2)
const uint8_t MAGIC_PNG[4] = {'P', 'X', 'U', 'C'};
const uint8_t PNG_VERSION = 0x01;
const size_t PNG_HEADER_SIZE = 11;  // MAGIC_RUN (4) + version (1) + frame_id (4) + length (2)

// Color configuration (adjust if colors appear swapped)
bool swapBytesSetting = false;  // keep false; colors are provided as RGB565 little-endian
bool useBgrSetting   = false;    // many ST7789 panels are BGR wired

// Stats
unsigned long frameCount = 0;
unsigned long lastStats = 0;
unsigned long updatesApplied = 0;
uint32_t lastFrameId = 0;
unsigned long timeOfLastFrame = 0;

struct PixelUpdate {
  uint16_t x;
  uint16_t y;
  uint8_t len;    // for run packets
  uint16_t color;
};

uint8_t* entry = nullptr;
uint32_t bufferCapacity = 0;
bool dmaEnabled = false;
uint16_t* framebuffer = nullptr;

// Buttons (ezButton handles debounce and pull-up internally)
ezButton btnGreen(GREEN_BUTTON);
ezButton btnRed(RED_BUTTON);
ezButton btnBlue(BLUE_BUTTON);
ezButton btnYellow(YELLOW_BUTTON);

// Select mode state
bool selectMode = false;
int  selectIndex = 0;  // currently highlighted slot in select screen

void disconnectClient(int cn) {
  if (clients[cn]) {
    clients[cn].flush();
    clients[cn].stop();
    clients[cn] = WiFiClient();
    Serial.print("Stopped client ");
    Serial.println(cn);
  } else {
    Serial.print("Unable to stop client ");
    Serial.println(cn);
  }
}

bool ensureUpdateBuffer(uint32_t needed) {
  if (needed <= bufferCapacity && entry != nullptr) {
    return true;
  }
  if (entry) {
    free(entry);
  }
  uint8_t* tmp = (uint8_t*)malloc(needed);
  if (!tmp) {
    Serial.println("Switching to PSRAM for entry");
    tmp = (uint8_t*)ps_malloc(needed);
  }
  if (!tmp) {
    Serial.println("Failed to allocate update buffer");
    return false;
  }
  entry = tmp;
  bufferCapacity = needed;
  return true;
}

bool readExactly(WiFiClient& c, uint8_t* dst, size_t len) {
  size_t got = 0;
  unsigned long start_t = millis();
  while (got < len && c.connected()) {
    int chunk = c.read(dst + got, len - got);
    if (chunk > 0) {
      got += chunk;
    } else {
      if (millis() - start_t > TIMEOUT) {
        return false;
      }
      delay(1);  // allow other tasks
    }
  }
  return got == len;
}

void applyColorConfig() {
  tft.setSwapBytes(swapBytesSetting);
  tft.writecommand(TFT_MADCTL);
  tft.writedata(useBgrSetting ? TFT_MADCTL_BGR : TFT_MADCTL_RGB);
}

void showWaitingScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 20);
  tft.setTextSize(2);
  tft.println("Pixel RX");
  tft.setCursor(10, 50);
  tft.setTextSize(1);
  tft.println("IP Address:");
  tft.setCursor(10, 70);
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println(WiFi.localIP().toString());
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 100);
  tft.setTextSize(1);
  tft.println("Waiting for");
  tft.setCursor(10, 115);
  tft.println("connection...");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Pixel Update Receiver ===");

  btnGreen.setDebounceTime(50);
  btnRed.setDebounceTime(50);
  btnBlue.setDebounceTime(50);
  btnYellow.setDebounceTime(50);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);  // backlight
  tft.init();
  SPI.setFrequency(SPI_TARGET_FREQ);
  dmaEnabled = tft.initDMA();
  tft.setRotation(0);  // portrait
  applyColorConfig();

  while (1) {
    tft.fillScreen(TFT_BLACK);
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
      delay(250);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFi connection failed");
      tft.fillScreen(TFT_RED);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.setCursor(10, 50);
      tft.setTextSize(1);
      tft.println("Failed to connect!");
      while (true) {
        delay(1000);
      }
    } else {
      break;
    }
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  if (dmaEnabled) {
    Serial.println("DMA enabled");
  }

  try {
    framebuffer = (uint16_t*)malloc(FRAMEBUFFER_SIZE << 1);
  }
  catch (int errorCode) {
    Serial.println("Unable to allocate framebuffer");
    abort();
  }
  memset(framebuffer, 0, FRAMEBUFFER_SIZE << 1);

  showWaitingScreen();

  server.begin();
  server.setNoDelay(true);
  Serial.println("Server listening on port 8090");

  for (int i = 0; i < MAX_CLIENTS; i++) {
    clients[i] = WiFiClient();
  }
}

void flushBuffer() {
  if (!clients[activeClient] || !clients[activeClient].connected()) {
    return;
  }
  while (clients[activeClient].available()) {
    clients[activeClient].read();
  }
}

void askForFrame() {
  if (!clients[activeClient] || !clients[activeClient].connected()) {
    return;
  }
  if (clients[activeClient].available()) {            // if got data, process it first
    return;
  }
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (i == activeClient) {
      clients[i].write(CLIENT_SEND);
    } else {
      if (clients[i] && clients[i].connected()) {
        if (!clients[i].write(CLIENT_WAIT)) {
          disconnectClient(i);
        }
      }
    }
  }
}

void askForFullFrame() {
  if (!clients[activeClient] || !clients[activeClient].connected()) {
    return;
  }
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (i == activeClient) {
      clients[i].write(CLIENT_SEND_FULL, 6);
      Serial.println("ask for full frame");
    } else {
      if (clients[i] && clients[i].connected()) {
        if (!clients[i].write(CLIENT_WAIT)) {
          disconnectClient(i);
        }
      }
    }
  }
  timeOfLastFrame = millis();
}

void switchClient() {
  Serial.print("Switching client, old client = ");
  Serial.println(activeClient);
  if (activeClient < 0) return;

  int start = activeClient;
  do {
    activeClient++;
    if (activeClient >= MAX_CLIENTS)
      activeClient = 0;

    if (clients[activeClient] && clients[activeClient].connected()) {
      Serial.print("Switched to client ");
      Serial.println(activeClient);
      memset(framebuffer, 0, FRAMEBUFFER_SIZE << 1);        // clear buffer
      flushBuffer();
      askForFullFrame();
      return;
    }
  } while (activeClient != start);
  activeClient = -1;
  Serial.println("No active clients");
}

// remove disconnected clients and potentially add new client
void acceptClients() {
  WiFiClient newClient = server.available();

  for (int i = 0; i < MAX_CLIENTS; i++) {                 // remove disconnected or duplicate clients from list
    if (clients[i] && !clients[i].connected()) {
      disconnectClient(i);                                  // destroy client object
      Serial.print("Client ");
      Serial.print(i);
      Serial.println(" disconnected");
      if (!selectMode) showWaitingScreen();
      if (activeClient == i) {
        switchClient();
      }
    }
  }

  if (newClient) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (!clients[i] || !clients[i].connected()) {
        clients[i] = newClient;
        clients[i].setNoDelay(true);
        clients[i].setTimeout(50);

        Serial.print("Client added at slot ");
        Serial.println(i);

        if (activeClient == -1) {
          activeClient = i;
          askForFullFrame();
        } 

        return;
      }
    }

    Serial.println("Client list full");
    newClient.stop();
  }
}

void showSelectScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);

  // Title bar
  tft.fillRect(0, 0, DISPLAY_WIDTH, 14, TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setCursor(4, 3);
  tft.print("Select Client");

  // Column headers
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(4, 18);
  tft.print("Slot  IP Address       St");

  // Draw each slot
  for (int i = 0; i < MAX_CLIENTS; i++) {
    int y = 30 + i * 18;
    bool isHighlighted = (i == selectIndex);
    bool isActive      = (i == activeClient);
    bool isConnected   = (clients[i] && clients[i].connected());

    // Row background
    if (isHighlighted) {
      tft.fillRect(0, y - 2, DISPLAY_WIDTH, 16, TFT_NAVY);
    } else {
      tft.fillRect(0, y - 2, DISPLAY_WIDTH, 16, TFT_BLACK);
    }

    // Highlight arrow
    tft.setTextColor(isHighlighted ? TFT_YELLOW : TFT_BLACK,
                     isHighlighted ? TFT_NAVY   : TFT_BLACK);
    tft.setCursor(0, y);
    tft.print(isHighlighted ? ">" : " ");

    // Slot number, bold-ish via active marker
    tft.setTextColor(isActive ? TFT_GREEN : TFT_WHITE,
                     isHighlighted ? TFT_NAVY : TFT_BLACK);
    tft.setCursor(8, y);
    tft.printf("[%d]", i + 1);

    // IP or "empty"
    tft.setCursor(30, y);
    if (isConnected) {
      tft.setTextColor(TFT_WHITE, isHighlighted ? TFT_NAVY : TFT_BLACK);
      tft.print(clients[i].remoteIP().toString().substring(0, 13));
    } else {
      tft.setTextColor(TFT_DARKGREY, isHighlighted ? TFT_NAVY : TFT_BLACK);
      tft.print("empty        ");
    }

    // Status dot
    tft.setCursor(116, y);
    if (isConnected) {
      tft.setTextColor(TFT_GREEN, isHighlighted ? TFT_NAVY : TFT_BLACK);
      tft.print("*");
    } else {
      tft.setTextColor(TFT_DARKGREY, isHighlighted ? TFT_NAVY : TFT_BLACK);
      tft.print("-");
    }
  }

  // Button hint at bottom
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(0, 125);
  tft.print("B/Y: up/down\nG: delect R: cancel");
}

void enterSelectMode() {
  selectMode = true;
  // Start highlight on current active client, or 0
  selectIndex = (activeClient >= 0) ? activeClient : 0;
  showSelectScreen();
}

void exitSelectMode(bool cancelled) {
  selectMode = false;
  if (!cancelled && clients[selectIndex] && clients[selectIndex].connected()) {
    // Switch to selected client
    activeClient = selectIndex;
    memset(framebuffer, 0, FRAMEBUFFER_SIZE << 1);
    tft.fillScreen(TFT_BLACK);
    flushBuffer();
    askForFullFrame();
  }
  // Redraw whatever is appropriate
  if (activeClient < 0) {
    showWaitingScreen();
  }
  // else the next frame push will overwrite the screen naturally
}

void checkButton() {
  btnGreen.loop();
  btnRed.loop();
  btnBlue.loop();
  btnYellow.loop();

  if (selectMode) {
    if (btnRed.isPressed()) {
      exitSelectMode(true);               // quit screen
    }
    else if (btnGreen.isPressed()) {
      exitSelectMode(false);              // switch to selected client
    }
    else if (btnBlue.isPressed()) {
      selectIndex = (selectIndex + 1) % MAX_CLIENTS;
      showSelectScreen();
    }
    else if (btnYellow.isPressed()) {
      selectIndex = (selectIndex - 1 + MAX_CLIENTS) % MAX_CLIENTS;
      showSelectScreen();
    }
  } else {
    if (btnRed.isPressed()) {
      enterSelectMode();
    }
  }
}

bool decodePixelFrame(WiFiClient& client) {
  uint8_t rest[HEADER_SIZE - 4];
  if (!readExactly(client, rest, sizeof(rest))) {
    Serial.println("Failed to read pixel header; dropping client");
    disconnectClient(activeClient);
    return false;
  }
  if (rest[0] != PROTO_VERSION) {
    Serial.print("Unsupported pixel version: ");
    Serial.println(rest[0], HEX);
    disconnectClient(activeClient);
    return false;
  }

  uint32_t frameId = ((uint32_t)rest[1]) | ((uint32_t)rest[2] << 8) | ((uint32_t)rest[3] << 16) | ((uint32_t)rest[4] << 24);
  uint16_t count = rest[5] | (rest[6] << 8);  // little-endian
  if (count == 0) {
    frameCount++;
    lastFrameId = frameId;
    return true;
  }
  if (count > (DISPLAY_WIDTH * DISPLAY_HEIGHT)) {
    Serial.print("Update count too large: ");
    Serial.println(count);
    disconnectClient(activeClient);
    return false;
  }

  if (!ensureUpdateBuffer(count * sizeof(PixelUpdate))) {
    Serial.println("No buffer for updates; dropping client");
    disconnectClient(activeClient);
    return false;
  }

  //Serial.print("Reading ");
  //Serial.print(count);
  //Serial.println(" pixels");
  if (!readExactly(client, entry, 6 * count)) {
    Serial.println("Stream ended mid-frame; dropping client");
    disconnectClient(activeClient);
    return false;
  }

  // Apply all updates in one batch after the full frame is received
  for (uint16_t i = 0; i < count; i++) {
    uint32_t base = i * 6;
    uint16_t x = entry[base + 0] | entry[base + 1] << 8;
    uint16_t y = entry[base + 2] | entry[base + 3] << 8;
    if (x < DISPLAY_WIDTH && y < DISPLAY_HEIGHT) {
      framebuffer[x + y * DISPLAY_WIDTH] = entry[base + 4] << 8 | entry[base + 5];
      updatesApplied++;
    }
  }

  tft.startWrite();
  tft.pushImage(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, framebuffer);
  tft.endWrite();

  frameCount++;
  lastFrameId = frameId;
  unsigned long now = millis();
  if (now - lastStats > 2000) {
    Serial.print("Frames: ");
    Serial.print(frameCount);
    Serial.print(" (last frameId ");
    Serial.print(lastFrameId);
    Serial.print(") | Updates applied: ");
    Serial.println(updatesApplied);
    lastStats = now;
  }
  return true;
}

bool decodeRunFrame(WiFiClient& client) {
  uint8_t rest[RUN_HEADER_SIZE - 4];
  if (!readExactly(client, rest, sizeof(rest))) {
    Serial.println("Failed to read run header; dropping client");
    disconnectClient(activeClient);
    return false;
  }
  if (rest[0] != RUN_VERSION) {
    Serial.print("Unsupported run version: ");
    Serial.println(rest[0], HEX);
    disconnectClient(activeClient);
    return false;
  }

  uint32_t frameId = ((uint32_t)rest[1]) | ((uint32_t)rest[2] << 8) | ((uint32_t)rest[3] << 16) | ((uint32_t)rest[4] << 24);
  uint16_t count = rest[5] | (rest[6] << 8);  // number of runs
  if (count == 0) {
    frameCount++;
    lastFrameId = frameId;
    return true;
  }
  if (count > (DISPLAY_WIDTH * DISPLAY_HEIGHT)) {
    Serial.print("Run count too large: ");
    Serial.println(count);
    disconnectClient(activeClient);
    return false;
  }

  //Serial.print("Reading ");
  //Serial.print(count);
  //Serial.println(" RLE pixels");
  if (!ensureUpdateBuffer(count * sizeof(PixelUpdate))) {
    Serial.println("No buffer for run updates; dropping client");
    disconnectClient(activeClient);
    return false;
  }

  // Each run entry: y (2), x0 (2), length (1), color (2) = 7 bytes
  if (!readExactly(client, entry, 7 * count)) {
    Serial.println("Stream ended mid-run frame; dropping client");
    disconnectClient(activeClient);
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    uint16_t base = i * 7;
    uint16_t color = entry[base + 5] << 8 | entry[base + 6];
    uint32_t x0 = entry[base + 2] | entry[base + 3] << 8;
    uint16_t y = entry[base + 0] | entry[base + 1] << 8;
    uint8_t runLen = entry[base + 4];
    if (x0 < DISPLAY_WIDTH && y < DISPLAY_HEIGHT && runLen > 0 && (x0 + runLen) <= DISPLAY_WIDTH) {
      x0 += y * DISPLAY_WIDTH;
      for (uint8_t i = 0; i < runLen; i++) {
        framebuffer[x0++] = color;
      }
      updatesApplied += runLen;
    }
  }

  tft.startWrite();
  tft.pushImage(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, framebuffer);
  tft.endWrite();

  frameCount++;
  lastFrameId = frameId;
  unsigned long now = millis();
  if (now - lastStats > 2000) {
    Serial.print("Frames: ");
    Serial.print(frameCount);
    Serial.print(" (last frameId ");
    Serial.print(lastFrameId);
    Serial.print(") | Updates applied: ");
    Serial.println(updatesApplied);
    lastStats = now;
  }

  return true;
}

int decodeLine(PNGDRAW *pDraw) {
  uint16_t line[DISPLAY_WIDTH];
  uint16_t *ptr;
  png.getLineAsRGB565(pDraw, line, PNG_RGB565_BIG_ENDIAN, PNG_NO_ALPHA);
  ptr = framebuffer + pDraw->y * DISPLAY_WIDTH;
  memcpy(ptr, line, DISPLAY_WIDTH << 1);
  return 1;
}

bool decodePngFrame(WiFiClient& client) {
  uint8_t rest[PNG_HEADER_SIZE - 4];
  if (!readExactly(client, rest, sizeof(rest))) {
    Serial.println("Failed to read PNG header; dropping client");
    disconnectClient(activeClient);
    return false;
  }
  if (rest[0] != PNG_VERSION) {
    Serial.print("Unsupported PNG version: ");
    Serial.println(rest[0], HEX);
    disconnectClient(activeClient);
    return false;
  }

  uint32_t frameId = ((uint32_t)rest[1]) | ((uint32_t)rest[2] << 8) | ((uint32_t)rest[3] << 16) | ((uint32_t)rest[4] << 24);
  uint16_t count = rest[5] | (rest[6] << 8);  // little-endian
  if (count == 0) {
    frameCount++;
    lastFrameId = frameId;
    return true;
  }

  if (!ensureUpdateBuffer(count)) {
    Serial.println("No buffer for updates; dropping client");
    disconnectClient(activeClient);
    return false;
  }

  //Serial.print("Reading ");
  //Serial.print(count);
  //Serial.println(" bytes");
  if (!readExactly(client, entry, count)) {
    Serial.println("Stream ended mid-frame; dropping client");
    disconnectClient(activeClient);
    return false;
  }

  int status = png.openRAM(entry, count, decodeLine);
  if (status == PNG_SUCCESS) {
    status = png.decode(NULL, 0);
    png.close();
    if (status != PNG_SUCCESS) {
      Serial.print("Error decoding PNG packet, error = ");
      Serial.println(status);
      disconnectClient(activeClient);
      return false;
    }

    tft.startWrite();
    tft.pushImage(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, framebuffer);
    tft.endWrite();

    frameCount++;
    lastFrameId = frameId;
    unsigned long now = millis();
    if (now - lastStats > 2000) {
      Serial.print("Frames: ");
      Serial.print(frameCount);
      Serial.print(" (last frameId ");
      Serial.print(lastFrameId);
      Serial.print(") | Updates applied: ");
      Serial.println(updatesApplied);
      lastStats = now;
    }
    return true;

  } else {
    Serial.print("Error opening PNG packet, status = ");
    Serial.println(status);
    disconnectClient(activeClient);
    return false;
  }
}

// return true if frame processed so need to ask for another one
bool handleClient() {
  WiFiClient &client = clients[activeClient];

  if (!client || !client.connected()) {
    switchClient();
    return false;
  }

  if ((millis() - timeOfLastFrame) > TIMEOUT) {
    Serial.println("Client timeout");
    flushBuffer();
    askForFullFrame();
  }

  // Require header to begin processing (pixel or run)
  if (client.available() < 11) {
    return false;
  }

  timeOfLastFrame = millis();

  // Peek magic to decide packet type
  uint8_t magicBuf[4];
  if (!readExactly(client, magicBuf, 4)) {
    disconnectClient(activeClient);
    return false;
  }
  bool isRun = (memcmp(magicBuf, MAGIC_RUN, 4) == 0);
  bool isPixel = (memcmp(magicBuf, MAGIC, 4) == 0);
  bool isPng = (memcmp(magicBuf, MAGIC_PNG, 4) == 0);

  if (isPixel) {
    return decodePixelFrame(client);
  }

  if (isRun) {
    return decodeRunFrame(client);
  }

  if (isPng) {
    return decodePngFrame(client);
  }

  Serial.println("Bad magic; dropping client");
  disconnectClient(activeClient);
  return false;
}

void flushClients() {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (i != activeClient) {
      if (clients[i] && clients[i].connected()) {
        clients[i].flush();
      }
    }
  }
}

void loop() {
  acceptClients();
  checkButton();

  if (!selectMode && activeClient >= 0)
    if (handleClient()) {
      askForFrame();
    }

  flushClients();

}
