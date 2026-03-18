# ESP32 Desktop Monitor

Stream your computer screen or other video source to an ESP32 T-Display over WiFi. This project enables you to mirror your monitor to a small LCD display connected to an ESP32.

## Hardware Requirements

### ESP32 Development Board
- **Board**: TENSTAR T-Display ESP32-D0WD (or compatible ESP32 with integrated display)
- **Chip**: ESP32-D0WD with CH9102 USB-to-Serial chip
- **Memory**: 16MB Flash
- **Display**: 1.14" ST7789 LCD (135x240 pixels)
- **Connectivity**: WiFi and Bluetooth compatible

### Computer
- Any computer running Python 3.7+
- macOS, Linux, or Windows (with appropriate screen capture libraries)
- Video can be captured from HDMI, DSI (Raspberry PI) or framebuffer

## Software Requirements

### ESP32 Side (Arduino IDE)
1. **Arduino IDE** (1.8.x or 2.x) or **PlatformIO**
2. **ESP32 Board Support Package**
   - Add this URL to Arduino IDE Preferences → Additional Board Manager URLs:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Install "esp32" by Espressif Systems from Board Manager
3. **TFT_eSPI Library**
   - Install via Arduino Library Manager
   - Or download from: https://github.com/Bodmer/TFT_eSPI
   - **Important**: Configure the library for your display (see TFT_eSPI setup guide)

### Computer Side (Python)
- Python 3.7 or higher
- Required packages (install via `pip install -r requirements.txt`):
  - `opencv-python` - Image processing and scaling
  - `mss` - Cross-platform screen capture (for HDMI capture)
  - `numpy` - Array operations
  - `PIL` - Image capture (for DSI)
  - `subprocess` - Running OS commands (for DSI)
  - `io` - Core tools for working with streams (for DSI)
  - `os` - Miscellaneous operating system interfaces (for framebuffer)

## Setup Instructions

### 1. Configure ESP32 Receiver

1. Open `receiver.ino` in Arduino IDE
2. Update WiFi credentials:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Configure TFT_eSPI library:
   - Open `User_Setup.h` in the TFT_eSPI library folder
   - Ensure your display driver (ST7789) is selected
   - Verify pin definitions match your board
   - For T-Display boards, typically:
     - Driver: ST7789
     - Width: 135, Height: 240
     - Rotation: 0 (portrait)
4. Select board: **Tools → Board → ESP32 Dev Module** (or your specific ESP32 board)
5. Select port: **Tools → Port → [Your ESP32 port]**
6. Upload the sketch

### 2. Find ESP32 IP Address

After uploading, open the Serial Monitor (115200 baud). The ESP32 will:
- Connect to WiFi
- Display its IP address on the screen
- Print the IP address to Serial Monitor

Note the IP address (e.g., `192.168.1.100`).

### 3. Install Python Dependencies

```bash
pip install -r requirements.txt
```

**macOS users**: Grant "Screen Recording" permission to Terminal (or your Python environment) when prompted.

### 4. Run the Transmitter

```bash
python transmitter.py --ip <ESP32_IP_ADDRESS> --backend <backend video type>
```

Replace `<ESP32_IP_ADDRESS>` with the IP address from step 2.
Replace `<backend video type>` with your video capture type (windows/dsi/framebuffer)

## Usage

### Basic Usage

```bash
# Stream leftmost monitor to ESP32
python transmitter.py --ip 192.168.1.100 --backend windows

# Stream specific monitor (1-based index)
python transmitter.py --ip 192.168.1.100 --monitor-index 2 --backend windows

# Adjust frame rate (default: 15 FPS)
python transmitter.py --ip 192.168.1.100 --target-fps 20 --backend windows

# Adjust change detection sensitivity (default: 5)
python transmitter.py --ip 192.168.1.100 --threshold 8 --backend windows
```

Press the button on the ESP32 to cycle through the available clients (transmitters)

### Command Line Options

- `--ip <IP>` - ESP32 IP address (required)
- `--port <PORT>` - TCP port (default: 8090)
- `--monitor-index <N>` - Select monitor by index (1-based, default: leftmost)
- `--prefer-largest` - Use largest monitor instead of leftmost
- `--target-fps <FPS>` - Target frame rate (default: 15.0)
- `--threshold <N>` - Pixel change threshold 0-255 (default: 5, higher = less sensitive)
- `--full-frame` - Send all pixels every frame (no diffing, slower)
- `--max-updates-per-frame <N>` - Max pixels per packet (default: 3000)
- `--rotate <0|90|180|270>` - Rotate capture before scaling
- `--show-cursor` - Draw cursor on captured frame (macOS only)
- `--backend <windows|dsi|framebuffer>` - Video capture source

### Performance Tuning

For better frame rates:

1. **Increase threshold** (reduces bandwidth):
   ```bash
   python transmitter.py --ip 192.168.1.100 --threshold 8
   ```

2. **Adjust frame rate**:
   ```bash
   python transmitter.py --ip 192.168.1.100 --target-fps 20
   ```

3. **Increase packet size** (for high-motion content):
   ```bash
   python transmitter.py --ip 192.168.1.100 --max-updates-per-frame 8000
   ```

4. **ESP32 side**: If display is unstable, lower SPI frequency in `receiver.ino`:
   ```cpp
   const uint32_t SPI_TARGET_FREQ = 40000000;  // Lower from 80000000
   ```

## How It Works

### Protocol

The system uses a custom protocol optimized for small displays:

1. **Frame Diffing**: Only pixels that changed are sent (configurable threshold)
2. **Run-Length Encoding**: Consecutive pixels of the same color are encoded as runs
3. **Automatic Selection**: The sender chooses the most efficient encoding (pixel-by-pixel vs. run-length)
4. **Batched Updates**: All updates for a frame are received before applying to display
5. **Client start/stop commands**: Client told to only send updates when it is being displayed

### Packet Format

**Pixel Packets** (`PXUP`):
- Header: `'PXUP'` (4 bytes) + version (1 byte) + frame_id (4 bytes) + count (2 bytes)
- Body: `count` entries of `x` (1 byte), `y` (1 byte), `color` (2 bytes, RGB565)

**Run Packets** (`PXUR`):
- Header: `'PXUR'` (4 bytes) + version (1 byte) + frame_id (4 bytes) + count (2 bytes)
- Body: `count` entries of `y` (1 byte), `x0` (1 byte), `length` (1 byte), `color` (2 bytes, RGB565)

### Optimizations

- **High-Speed SPI**: 80MHz SPI clock (configurable, fallback to 40MHz if unstable)
- **DMA Support**: Uses DMA for efficient display updates when available
- **PSRAM Usage**: Leverages ESP32 PSRAM for large update buffers
- **TCP_NODELAY**: Disables Nagle's algorithm for lower latency

## Troubleshooting

### Colors Appear Swapped

If colors look wrong, adjust in `receiver.ino`:
```cpp
bool useBgrSetting = false;  // Try changing from true to false
```

### Low Frame Rate

1. Check WiFi signal strength
2. Increase `--threshold` to reduce bandwidth
3. Lower `--target-fps` if network can't keep up
4. Ensure ESP32 and computer are on the same WiFi network

### Connection Issues

1. Verify ESP32 IP address in Serial Monitor
2. Check firewall settings (port 8090)
3. Ensure both devices are on the same network
4. Try restarting the ESP32

### Display Not Working

1. Verify TFT_eSPI library configuration
2. Check pin definitions in `User_Setup.h`
3. Ensure backlight pin is correct (typically pin 4)
4. Lower SPI frequency if display is unstable

## License

This project is provided as-is for educational and personal use.

## Contributing

Contributions welcome! Please feel free to submit issues or pull requests.

## Acknowledgments

- TFT_eSPI library by Bodmer
- ESP32 Arduino core by Espressif
- mss library for cross-platform screen capture

