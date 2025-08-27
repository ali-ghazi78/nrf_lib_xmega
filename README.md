# My First XMEGA Project

This project demonstrates peripheral programming on an **Atmel XMEGA microcontroller** using C.  
It integrates multiple hardware interfaces such as **USART, SPI, I²C (TWI), PWM, GPIO, and external RF modules (nRF24L01)** to test and showcase the capabilities of the XMEGA platform.  

---

## ✨ Features

- **System Clock**
  - Configures the XMEGA to run at 32 MHz with peripheral prescalers.

- **GPIO (LEDs & Keys)**
  - Four LEDs (UP, DOWN, LEFT, RIGHT) for visual feedback.
  - Four input buttons mapped to directions.

- **USART**
  - 115200 baud, 8N1 format.
  - Polling and interrupt-driven modes.
  - Functions for characters, strings, and integers.
  - ASCII table printout demo.

- **I²C (TWI)**
  - Start, stop, read, and write support.
  - Example: OLED and compass sensor communication.

- **SPI**
  - Master mode using `SPID`.
  - Used to interface with **nRF24L01** wireless modules.

- **nRF24L01 Wireless**
  - Initialization for TX and RX.
  - Load and transmit payloads.
  - Receive and process incoming data.
  - FIFO and IRQ flag handling.

- **PWM**
  - Configured on `PORTE`.
  - Example: ~38 kHz signal with adjustable duty cycle.

- **Helper Functions**
  - LED test sequence.
  - Key test routine.
  - OLED text output.
  - Compass calibration.

---

## 🛠 Requirements

### Hardware
- Atmel XMEGA microcontroller (e.g., XMEGA128A1).  
- 4× LEDs, 4× push buttons.  
- nRF24L01 RF module(s).  
- Optional: OLED display, MPU/compass sensor.  

### Software / Toolchain
- AVR-GCC / Atmel Studio (Microchip Studio).  
- AVRDUDE or Atmel ICE / USBasp programmer.  

---

## 🚀 How It Works

1. **Initialization**
   - Clock, pins, USART, SPI, and I²C initialized.
   - LEDs blink in startup sequence.

2. **Communication**
   - USART prints debug messages.  
   - SPI initializes nRF24 in both TX and RX modes.  
   - I²C can talk to OLED/compass.  

3. **Main Loop**
   - Transmits incrementing bytes over **nRF24 TX**.  
   - Receives data over **nRF24 RX**.  
   - LEDs indicate status.  

---

## 📂 File Structure

