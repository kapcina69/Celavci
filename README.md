# Celavci

Firmware projekat za **nRF52840** bazirani modul, razvijen u okviru **Zephyr RTOS-a**.

## Funkcionalnosti
- **BLE komunikacija** preko Nordic UART Service (NUS)
- **ADC očitavanja** (npr. merenje napona i struje)
- **DAC kontrola** (MCP4716, I2C magistrala)
- **Praćenje baterije** (TI BQ27220 fuel gauge)
- **Multiplexer kontrola** (HV2607, SPI magistrala)
- **GPIO kontrola** (katoda, anoda, DC-DC uključivanje)

## Maksimalna učestanost u zavisnosti od dužine pulsa

Tabela pokazuje maksimalnu dozvoljenu frekvenciju (Hz) u odnosu na dužinu impulsa (µs):

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

---

📌 Primer:  
- Na dužini impulsa od **350 µs** maksimalna učestanost je **100 Hz**.  



## Struktura projekta
- `CMakeLists.txt` – build konfiguracija
- `mymodule.dts` – definicija hardverskih resursa (LED, GPIO, I2C, SPI, ADC, PWM)
- `src/` – C fajlovi sa implementacijom
- `include/` – zaglavlja sa interfejsima

## Build i flešovanje
```bash
west build -b nrf52840dk_nrf52840 .
west flash
