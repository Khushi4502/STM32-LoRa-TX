#  STM32 LoRa TX Example

This project demonstrates how to use **STM32L051C8Tx** with an **EByte LoRa module (E32/E220)** for **data transmission (TX)**.  
It continuously sends packets with an incrementing counter and prints debug messages over UART.

---

##  Hardware

- **MCU**: STM32L051C8Tx (tested)  
- **LoRa Module**: EByte E32/E220 (433/868/915 MHz)  
- **LEDs**: Indicators for TX/RX status  
- **Power**: 3.3V  

---

##  Pin Connections

| STM32 Pin | Function      | LoRa Pin |
|-----------|--------------|----------|
| PA2       | USART2_TX    | RXD      |
| PA3       | USART2_RX    | TXD      |
| PB14      | MODE0 (M0)   | M0       |
| PB4       | MODE1 (M1)   | M1       |
| PB15      | AUX (INT)    | AUX      |
| 3.3V      | VCC          | VCC      |
| GND       | GND          | GND      |

---

##  Pin Diagram



*(This shows the STM32L051C8Tx with LoRa connections)*

---

## Features

- Sends a 4-byte packet every 8 seconds  
- Packet includes a **counter value** that increments each cycle  
- LED1 → ON during transmission  
- LED2 → Blinks if response received from LoRa receiver  
- Debug messages over UART2 (for monitoring on serial terminal)  

---
