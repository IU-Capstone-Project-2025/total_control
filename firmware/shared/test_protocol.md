# Communication Protocol v1.0

## Overview
Binary protocol for Arduino-Python communication over serial (UART/USB).  
Baud rate: **115200**

## JSON Protocol
```json
{
  "cmd": "set_led",
  "payload": {
    "pin": 13,
    "state": true
  },
  "timestamp": 123456789
}
