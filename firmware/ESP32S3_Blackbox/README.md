# EDC2026 ESP32-S3 Blackbox

ESP32-S3 telemetry blackbox for the MSPM0 line-following car.

## Features

- UART telemetry input from MSPM0
- WiFi AP mode
- Browser dashboard at `http://192.168.4.1/`
- Raw telemetry log saved to SPIFFS
- Log download and clear endpoints
- UDP broadcast on port `3333`

## Default Wiring

| ESP32-S3 | MSPM0 |
| --- | --- |
| GPIO18 RX | MSPM0 UART0 TX / PB0 |
| GPIO17 TX | optional MSPM0 UART0 RX / PB1 |
| GND | GND |

Default UART baud rate: `115200`.

## WiFi

- SSID: `EDC2026_BLACKBOX`
- Password: `edc2026car`
- Web UI: `http://192.168.4.1/`

## Build

Open this folder in VS Code with the ESP-IDF extension:

```text
D:\PycharmProjects\ESP32\EDC2026
```

Then run:

```powershell
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Telemetry Protocol

Version 1 accepts any newline-terminated text line from MSPM0.

Current MSPM0 line format:

```text
T=123456,TASK=2,LAP=0,MODE=1,BC=0,YAW=472,H=468,X=123,Y=-5,S=130,D=118,V=74,VF=1234,ER=2
```

The ESP32 logs the raw line, exposes it in the browser, and broadcasts it over UDP.

Scaled fields:

- `YAW` and `H` are degrees x10
- `X`, `Y`, `S`, `D` are millimeters
- `V` is battery voltage x10
