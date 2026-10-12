 
# ESP32-C3 WSPR Beacon Firmware

This firmware features exact UTC time synchronization via GPS, millihertz-level RF frequency control using an **Si5351** clock generator, and automated RF path routing/band switching controlled by a **PCF8574** I2C GPIO extender driving an external RF switch matrix[cite: 1].

---

## Features

* **GPS UTC Synchronization:** Automatic system time lock using NMEA `$GPRMC` parsing synchronized with the 1 PPS hardware pin.
* **Exact WSPR Type 1 Encoding:** Standard 162-symbol WSPR frame generation (50 message bits + 31 tail bits with convolutional encoding and interleaving).

## Hardware Configuration & Pinout

### ESP32-C3 Pin Mapping

| Component | Function | ESP32-C3 Pin |
| :--- | :--- | :--- |
| **I2C Bus** | SDA | `GPIO 5` |
| **I2C Bus** | SCL | `GPIO 6` |
| **GPS Module** | RX (To GPS TX) | `GPIO 20` |
| **GPS Module** | TX (To GPS RX) | `GPIO 21` |
| **GPS Module** | PPS | `GPIO 10` |

---

## PCF8574 RF Switch Control Matrix

The PCF8574 GPIO extender sets the control lines ($P1$ and $P2$) for the RF switch matrix to select low-pass filter paths:

| Mode / Band | Dial Frequency | PCF8574 Byte | Bit Logic ($P2, P1$) | Cutoff Frequency |
| :--- | :--- | :--- | :--- | :--- |
| **30m Band** | 10.1387 MHz | `0x06` | `P2=1, P1=1` | 15.5 MHz |
| **20m Band** | 14.0956 MHz | `0x06` | `P2=1, P1=1` | 15.5 MHz |
| **17m Band** | 18.1046 MHz | `0x02` | `P2=0, P1=1` | 29.0 MHz |
| **15m Band** | 21.0946 MHz | `0x02` | `P2=0, P1=1` | 29.0 MHz |
| **12m Band** | 24.9246 MHz | `0x04` | `P2=1, P1=0` | 32.0 MHz |
| **10m Band** | 28.1246 MHz | `0x04` | `P2=1, P1=0` | 32.0 MHz |
| **Bypass** | Direct Trace | `0x00` | `P2=0, P1=0` | Unfiltered |

---

## Configuration

Before flashing, update your station details at the top of `main/main_multiband.c`:

```c
#define CALLSIGN        "K1ABC"     // Your Call Sign
#define GRID_SQUARE     "FM19"      // 4-character Maidenhead Grid Square
#define POWER_DBM       23          // Transmit power in dBm (23 dBm = ~200mW)
#define XTAL_FREQ_HZ    25000000ULL // Si5351 Crystal Frequency (25MHz or 27MHz)
