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



source ~/.espressif/v5.5.2/esp-idf/export.sh



eth_hello/
├── CMakeLists.txt
├── partitions_ota.csv
└── main/
    ├── CMakeLists.txt
    └── eth_hello.c



 idf.py menuconfig
Serial flasher config → Flash size → 8 MB
Partition Table → Custom partition CSV → partitions_ota.csv


Archivo partitions_ota.csv:
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x6000
otadata,  data, ota,     0xf000,  0x2000
phy_init, data, phy,     0x11000, 0x1000
ota_0,    app,  ota_0,   0x20000, 1M
ota_1,    app,  ota_1,   ,        1M


ls /dev/cu.*

/dev/cu.usbserial-110


idf.py -p /dev/cu.usbserial-110 flash monitor

Ctrl + ]	

IP      : 192.168.10.99
GW      : 192.168.10.100
MASK    : 255.255.255.0

curl http://192.168.10.99/	

hello

curl http://192.168.10.99/status

{
  "ok": true,
  "ip": "192.168.10.99",
  "gw": "192.168.10.100",
  "netmask": "255.255.255.0",
  "running_partition": "ota_0"
}


curl -X POST --data-binary @build/eth_hello.bin http://192.168.10.99/update



---

# 🚀 Subirlo a GitHub

Desde tu repo local:

```bash
git add README.md
git commit -m "Add README for ETH OTA project"
git push