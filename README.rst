# Celavci

Firmware projekat za **nRF52840** bazirani modul, razvijen u okviru **Zephyr RTOS-a**.

## Funkcionalnosti
- **BLE komunikacija** preko Nordic UART Service (NUS)
- **ADC očitavanja** (npr. merenje napona i struje)
- **DAC kontrola** (MCP4716, I2C magistrala)
- **Praćenje baterije** (TI BQ27220 fuel gauge)
- **Multiplexer kontrola** (HV2607, SPI magistrala)
- **GPIO kontrola** (katoda, anoda, DC-DC uključivanje)

## Struktura projekta
- `CMakeLists.txt` – build konfiguracija
- `mymodule.dts` – definicija hardverskih resursa (LED, GPIO, I2C, SPI, ADC, PWM)
- `src/` – C fajlovi sa implementacijom
- `include/` – zaglavlja sa interfejsima

## Build i flešovanje
```bash
west build -b nrf52840dk_nrf52840 .
west flash
