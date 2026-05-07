# ESP32-ETH02 Ethernet + OTA + Web Dashboard + Sensors + WiFi AP

Advanced project for ESP32 (based on the ESP32-ETH02 / LAN8720 board) designed to operate as a low-power sensor station with dual connectivity (Ethernet and WiFi) and remote management.

## ✨ Key Features

- 🌐 **Ethernet Connectivity:** Wired connection (RMII) with an assigned static IP (192.168.10.99).
- 📶 **WiFi AP + NAT:** Acts as a WiFi Access Point (`GoPro-AP`), routing traffic to the Ethernet connection.
- 🌡️ **I2C Sensors:** Reads temperature, humidity, and atmospheric pressure using AHT20 and BMP280 sensors.
- 🔋 **Power Management (Low Power):** Configurable duty cycles with *Deep Sleep* for energy saving, monitoring inactivity and link loss.
- 📊 **Interactive Web Dashboard:** Web interface hosted directly on the ESP32 with real-time charts (Chart.js) and system metrics.
- 🔄 **OTA (Over-The-Air):** Firmware updates via HTTP POST requests, without the need for cables.

---

## 📦 Hardware & Connections

- **Base board:** ESP32-ETH02 (or ESP32-WROOM-32E with a LAN8720 module).
- **I2C Sensors:**
  - **SDA:** GPIO 14
  - **SCL:** GPIO 15
- **Ethernet Control:**
  - **PHY Power:** GPIO 16 (Used to power off/on the PHY before entering Deep Sleep).
  - **MDC / MDIO:** GPIO 23 / GPIO 18.

---

## ⚙️ Network Configuration

The device brings up two network interfaces with the following default configuration:

**Ethernet (Static)**
- **IP:** `192.168.10.99`
- **Gateway:** `192.168.10.100`
- **Subnet Mask:** `255.255.255.0`

**WiFi Access Point**
- **SSID:** `GoPro-AP`
- **Password:** `GoPro123`
- **AP IP:** `192.168.4.1`

---

## 📡 HTTP Endpoints (API and Web)

The web server runs on port 80 and exposes the following routes:

- `GET /` : Displays the interactive HTML Web Dashboard.
- `GET /status` : Returns a JSON with system status, network details, boot partition, and low-power parameters.
- `GET /sensors` : Returns a JSON with the latest data read from the AHT20 and BMP280 sensors.
- `POST /update` : Endpoint to upload the `.bin` file and flash the firmware via OTA.

---

## 🧰 Requirements & Setup (ESP-IDF)

You need to have the **ESP-IDF** framework installed (version 5.x recommended).

### 1. Environment Setup
Activate the Espressif tools in your terminal:
```bash
source ~/.espressif/v5.5.2/esp-idf/export.sh