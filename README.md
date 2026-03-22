# ESP32-ETH02 Ethernet OTA Hello

Proyecto mínimo para ESP32 (ESP32-WROOM-32E + LAN8720 / ETH02) que incluye:

- 🌐 Servidor HTTP por Ethernet
- 📡 IP estática
- 🔄 Actualización OTA por HTTP (curl)
- 🧪 Endpoint `/status` con info del sistema

---

## 📦 Hardware

- ESP32-ETH02 (LAN8720)
- ESP32-WROOM-32E
- Conexión Ethernet (RMII)
- USB-UART para flasheo

---

## ⚙️ Requisitos

- ESP-IDF ≥ 5.x
- Python 3
- macOS / Linux

---

## 🧰 Setup ESP-IDF

Si no lo tienes instalado:

```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh