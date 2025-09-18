# Celavci - Electrical Stimulation Device Firmware

Firmware projekat za **nRF52840** baziran stimulator, razvijen u okviru **Zephyr RTOS-a**.

## Pregled

**Celavci** je napredni uređaj za električnu stimulaciju sa BLE komunikacijom, dizajniran za preciznu kontrolu bifaznih impulsa kroz 16-kanalni multiplekser. Uređaj kombinuje napredne algoritme za stimulaciju sa bezbednosnim funkcijama i real-time monitoring sistemom.

## Ključne funkcionalnosti

### 🔋 **Napajanje i Upravljanje**
- **Fuel Gauge**: TI BQ27220 za praćenje baterije
- **DC-DC konverter**: Efikasno upravljanje napajanjem
- **LTC2950**: Bezbednosni power management

### ⚡ **Električna stimulacija**
- **Bifazni impulsi**: Anode/Katode fazni stimulacioni protokol
- **16-kanalni MUX**: HV2607 SPI multiplekser za adresiranje kanala
- **10-bit DAC**: MCP4716 I2C za preciznu amplitude kontrolu
- **Frequency range**: 1-100 Hz sa pametnim ograničenjima
- **Pulse width**: 50-1000 µs sa automatskim validiranjem

### 📡 **BLE Komunikacija**
- **Nordic UART Service (NUS)**: Glavni komunikacioni kanal
- **Napredni protokol**: Binary frame format `>CMD;payload<`
- **Real-time control**: Trenutno upravljanje parametrima stimulacije
- **Status reporting**: Potpun sistem za praćenje stanja

### 🎛️ **PWM Kontrola**
- **Dual PWM**: Nezavisni kanali za različite funkcije
  - **PWM0 (P0.31)**: 10 kHz @ 40% - Start/stop signalizacija
  - **PWM1 (P0.28)**: 8 kHz @ 1% - Feedback kontrola

### 📊 **Senzori i Monitoring**
- **12-bit SAADC**: Precizno merenje sa GPPI integracijom
- **Real-time feedback**: Instant detekcija kontakta
- **Burst monitoring**: 8-pulse batch analiza
- **Adaptivna kontrola**: Automatsko prilagođavanje na osnovu feedback-a

## Maksimalna učestanost u zavisnosti od dužine pulsa

| Dužina pulsa (µs) | Maksimalna frekvencija (Hz) |
|-------------------|-----------------------------|
| 1000 µs           | 66 Hz  |
| 950 µs            | 69 Hz  |
| 900 µs            | 70 Hz  |
| 850 µs            | 73 Hz  |
| 800 µs            | 74 Hz  |
| 750 µs            | 77 Hz  |
| 700 µs            | 79 Hz  |
| 650 µs            | 83 Hz  |
| 600 µs            | 84 Hz  |
| 550 µs            | 88 Hz  |
| 500 µs            | 91 Hz  |
| 450 µs            | 95 Hz  |
| 400 µs            | 98 Hz  |
| < 400 µs          | 1 – 100 Hz |

> **Napomena**: Sistem automatski validira i ograničava kombinacije frekvencije i pulse width radi bezbednosti.

## Struktura projekta

```
Celavci/
├── boards/arm/mymodule/        # Board definition files
│   ├── mymodule.dts           # Device tree with pin assignments
│   ├── Kconfig.board          # Board configuration
│   └── mymodule_defconfig     # Default board config
├── src/                       # Source files
│   ├── main.c                # Main application entry
│   ├── ble_nus.c             # BLE communication handler
│   ├── impulse.c             # Stimulation pulse generator
│   ├── dac.c                 # DAC control module
│   ├── mux.c                 # Multiplexer control
│   ├── fuel_gauge.c          # Battery monitoring
│   ├── nrfx_adc.c           # ADC feedback system
│   └── pwm.c                 # PWM control module
├── include/                   # Header files
│   ├── ble_nus.h             # BLE interface definitions
│   ├── impulse.h             # Pulse generation API
│   ├── dac.h                 # DAC interface
│   ├── mux.h                 # MUX control API
│   ├── fuel_gauge.h          # Battery monitoring API
│   ├── nrfx_adc.h           # ADC interface
│   └── pwm.h                 # PWM control API
├── prj.conf                   # Project configuration
├── CMakeLists.txt            # Build configuration
└── README.md                 # This file
```

## BLE Komunikacioni protokol

### Format komandi
```
>CMD;payload<
```

### Dostupne komande

#### **Kontrola stimulacije**
- `>SON<` - Start stimulacije
- `>OFF<` - Stop stimulacije
- `>RCE<` - Remote Contact Enable (8 impulsa)

#### **Parametri stimulacije**
- `>SF;VV<` - Set Frequency (constant, VV = 1-100 Hz)
- `>SF;VV WW DD<` - Set Frequency (sweep, VV-WW Hz, DD sekundi)
- `>PW;HHLL<` - Set Pulse Width (16-bit, 50-1000 µs)
- `>ST;HHLL<` - Set Stimulation Time (16-bit, sekunde)

#### **Pattern kontrola**
- `>SA;16_bytes<` - Set Active pattern (8 channel pairs)
- `>SP;VV<` - Set Pattern index (0-15)
- `>XC;16_bytes<` - Cross-channel amplitude (8 pairs × 2 bytes)

#### **Status čitanje**
- `>RSS<` - Read System Status (kompletno stanje)
- `>RSC<` - Read State of Charge (baterija)

### Primer korišćenja
```
>SF;14<          # Set frequency to 20 Hz (0x14)
>PW;01F4<        # Set pulse width to 500 µs
>SA;0101020204040808101020204040808080<  # Set pattern
>SON<            # Start stimulation
>OFF<            # Stop stimulation
```

## Hardware konfiguracija

### GPIO Assignment
```
P0.02  - LED Green
P0.03  - LED Blue  
P0.30  - LED Red
P0.07  - Pulse Cathode
P0.15  - Pulse Anode
P0.04  - DC-DC Enable
P0.29  - PB MCU (button simulation)
P0.09  - KILL (LTC2950 control)
P0.28  - PWM1 Channel 0 (feedback)
P0.31  - PWM0 Channel 0 (buzzer/signal)
```

### I2C Bus (400kHz)
```
P0.26  - SDA
P0.27  - SCL

Devices:
- 0x63: MCP4716 DAC
- 0x55: BQ27220 Fuel Gauge
```

### SPI Bus (2MHz)
```
P0.06  - MOSI
P0.08  - SCK
P0.01  - CS (MUX)

Devices:
- HV2607 16-channel multiplexer
```

### ADC Input
```
P0.02  - AIN3 (feedback measurement)
```

## Build i programiranje

### Zavisnosti
```bash
# Nordic nRF Connect SDK
west update

# Ili direktno Zephyr
pip install west
west init -m https://github.com/zephyrproject-rtos/zephyr --mr main zephyrproject
cd zephyrproject
west update
```

### Build
```bash
# Za custom board
west build -b mymodule .

# Za nRF52840 DK (development)
west build -b nrf52840dk_nrf52840 .
```

### Programiranje
```bash
# J-Link
west flash --runner jlink

# nrfjprog
west flash --runner nrfjprog

# PyOCD  
west flash --runner pyocd
```

### Debug
```bash
# RTT logging (preporučeno)
JLinkRTTViewer

# GDB debugging
west debugserver
arm-none-eabi-gdb build/zephyr/zephyr.elf
```

## Bezbednosne funkcije

### Hardverska zaštita
- **DC-DC monitoring**: Automatsko isključivanje pri problemu
- **LTC2950 integration**: Hardware watchdog
- **Pulse validation**: Sprečavanje shoot-through kondicija

### Softverska zaštita  
- **Parameter validation**: Automatska provera granica
- **Frequency limits**: Tabela dozvoljenih kombinacija
- **Contact detection**: Real-time monitoring kontakta
- **Timeout protection**: Automatsko gašenje stimulacije

### Fail-safe mehanizmi
- **GPIO fail-safe**: Svi stimulacioni pinovi u LOW pri grešci
- **Watchdog timers**: Multi-level timeout zaštita
- **Error recovery**: Automatski restart subsistema

## Napredne funkcionalnosti

### Frequency Sweep
Sistemski podržan sweep između dve frekvencije:
```c
>SF;0A 1E 05<  // 10Hz -> 30Hz, menja svakih 5 sekundi
```

### Adaptive Amplitude
Cross-channel amplitude kontrola po kanalima:
```c
>XC;00C800C800C800C800C800C800C800C8<  // 200µA na svih 8 parova
```

### Pattern Sequencing
Do 16 različitih pattern-a sa cikličnim prebacivanjem.

### Real-time Feedback
SAADC sa GPPI integracijom za <1µs latenciju merenja.

## Performance karakteristike

- **Pulse precision**: ±1µs timing accuracy
- **Frequency range**: 1-100 Hz continuously variable
- **Amplitude resolution**: 10-bit (1024 levels)  
- **Pattern switching**: <100µs between patterns
- **BLE latency**: <20ms command response
- **Battery monitoring**: 1% SoC accuracy
- **Contact detection**: 100mV threshold with hysteresis

## Debugging i dijagnostika

### RTT Logging
```bash
# Real-time logging preko J-Link RTT
JLinkRTTLogger -Device NRF52840_XXAA -If SWD -Speed 4000 -RTTChannel 0
```

### BLE Debugging
- HEX/ASCII prikaz svih poruka
- Timestamping komandi
- Error kod tracking

### Hardware monitoring
- Fuel gauge periodic reporting
- GPIO state logging  
- Timer/PWM dijagnostika



---

**Verzija**: v1.0
**Datum**: 18.09.2025.  
**Platform**: nRF52840 + Zephyr RTOS
**Status**: Production Ready