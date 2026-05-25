# Quick Start Guide

THIS IS OUT OF DATE

## 5-Minute Setup

### Step 1: Flash ESP32 (One-time setup)

1. Open `receiver.ino` in Arduino IDE
2. Update WiFi credentials (lines 31-32):
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Upload to ESP32
4. Open Serial Monitor (115200 baud)
5. Note the IP address shown (e.g., `192.168.1.100`)

### Step 2: Install Python Dependencies

```bash
cd esp32-pixel-stream
pip install -r requirements.txt
```

### Step 3: Run!

```bash
python transmitter.py --ip <YOUR_ESP32_IP>
```

Replace `<YOUR_ESP32_IP>` with the IP from Step 1.

That's it! Your screen should now be streaming to the ESP32 display.

## Common Issues

**"Connection refused"**
- Check ESP32 IP address in Serial Monitor
- Ensure both devices are on the same WiFi network

**"Screen Recording permission" (macOS)**
- System Preferences → Security & Privacy → Privacy → Screen Recording
- Enable Terminal (or your Python app)

**Colors look wrong**
- Edit `receiver.ino`, change `useBgrSetting` from `true` to `false` (or vice versa)
- Re-upload to ESP32

**Low frame rate**
- Try: `python transmitter.py --ip <IP> --threshold 8 --target-fps 20`

