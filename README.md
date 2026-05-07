# ESP32-ETH02 Ethernet + OTA + Web Dashboard + Sensors + WiFi AP

This advanced project for the ESP32 (based on the ESP32-ETH02 / LAN8720 board) operates as a low-power sensor station with dual connectivity and remote management.

## ✨ Key Features

- 🌐 **Ethernet Connectivity:** Wired connection (RMII) with a static IP (192.168.10.99).
- 📶 **WiFi AP + NAT:** Built-in WiFi Access Point (`GoPro-AP`) that routes traffic to the Ethernet interface using NAT.
- 🌡️ **I2C Sensors:** Real-time data acquisition from AHT20 and BMP280 sensors (Temperature, Humidity, and Pressure).
- 🔋 **Power Management:** Deep Sleep cycles and automatic power-off for the Ethernet PHY to optimize energy consumption.
- 📊 **Interactive Web Dashboard:** A built-in web interface featuring live charts powered by Chart.js.
- 🔄 **OTA (Over-The-Air):** Wireless firmware updates via HTTP POST requests.

---

## 🚀 Firmware Update Guide (Step-by-Step)

Before starting either method, ensure you activate the Espressif development environment in your terminal:

source ~/.espressif/v5.5.2/esp-idf/export.sh

### Method 1: Update via USB (Wired)
Use this method for the initial flash or if the device is not reachable over the network.

1. Build the project: Generate the updated binary file.
   idf.py build
   
2. Identify the port: Find your board's serial port (e.g., /dev/cu.usbserial-110).
   ls /dev/cu.*
   
3. Flash and Monitor: Upload the code and open the serial console to view logs.
   idf.py -p /dev/cu.usbserial-110 flash monitor
   
   *Press Ctrl + ] to exit the monitor.*

### Method 2: Update via IP / Ethernet (OTA)
Use this method to update the device remotely once it is connected to your network.

1. Build the project: Ensure you have the latest binary in the build folder.
   idf.py build
   
2. Send firmware over the network: Use curl to upload the .bin file to the ESP32's update endpoint.
   curl -X POST --data-binary @build/eth_hello.bin http://192.168.10.99/update
   
   *The device will receive the file, switch the boot partition, and reboot automatically.*

---

## 📦 Hardware & Pinout

- **Board:** ESP32-ETH02 (LAN8720)
- **I2C Sensors:** SDA on GPIO 14, SCL on GPIO 15 (AHT20 and BMP280)
- **Ethernet Control:** GPIO 16 manages the PHY power state for low-power modes.
- **Ethernet MDC/MDIO:** GPIO 23 and GPIO 18.

## ⚙️ Default Network Configuration

- **Ethernet IP:** 192.168.10.99
- **Gateway:** 192.168.10.100
- **Subnet Mask:** 255.255.255.0
- **WiFi SSID:** GoPro-AP (Password: GoPro123)
- **WiFi AP IP:** 192.168.4.1

---

## 📡 API Endpoints

- GET / : Main Web Dashboard
- GET /status : System and network health
- GET /sensors : Raw sensor data in JSON format
- POST /update : OTA firmware upload endpoint

---

## 💾 Repository Maintenance

To save these changes and the new instructions to your Git repository:

git add .
git commit -m "Complete README update with step-by-step USB and OTA instructions"
git push