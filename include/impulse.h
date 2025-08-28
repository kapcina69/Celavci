/**
 * @file impulse.h
 * @brief Generisanje bifaznih (bipolarnih) stimulacionih impulsa preko GPIO i MUX.
 *
 * @details
 * Ovaj modul implementira **stroj stanja** (state machine) koji u pravilnom redosledu
 * upravlja izlazima *anode* i *katode* (GPIO) i istovremeno sekvencira uzorke (pattern)
 * na analognom multiplekseru (MUX) radi adresiranja kanala stimulacije.
 *
 * ### Pregled funkcionalnosti
 * - Inicijalizacija izlaznih pinova: **anoda**, **katoda**, **DC-DC enable**;
 * - Generisanje **bifaznog** impulsa: faza *anode ON* → faza *katode ON* → pauza;
 * - Nakon svakog kompletnog impulsa, **MUX** dobija sledeći pattern (2 bajta) da bi se ciklično
 *   izabrali naredni kanali (npr. 0x01, 0x02, 0x04, ... 0x80);
 * - Period ponavljanja sekvence zavisi od zadate **frekvencije** (podešava se preko BLE komandnog sloja).
 *
 * ### Tajming i definicije
 * - Širina pojedinačne faze impulsa određena je makroom #STIMULATION_PULSE_WIDTH_US
 *   i množi se internim parametrom `pulse_width` (vidi BLE NUS komande).
 *   Efektivno trajanje jedne faze:  `T_phase = STIMULATION_PULSE_WIDTH_US * pulse_width` (µs).
 * - Jedan kompletan **bifazni** impuls sadrži dve faze (anoda → katoda) i pauzu,
 *   pa minimalni impuls bez pauze traje približno `2 * T_phase`.
 * - Period pobude računamo iz frekvencije: @ref hz_to_us(frequency_hz) (µs).
 *   U okviru implementacije se nakon svake sekvence raspoređuje sledeće aktiviranje work‑a
 *   tako da se poštuje zadati period (frekvencija).
 *
 * ### Stanja mašine (interno u .c)
 * - **PULSE_ANODE_ON**: anoda = 1, katoda = 0, traje `T_phase`;
 * - **PULSE_CATHODE_ON**: anoda = 0, katoda = 1, traje `T_phase`;
 * - **PULSE_PAUSE**: anoda = 0, katoda = 0; ažurira se MUX pattern i raspoređuje sledeći ciklus;
 * - **PULSE_IDLE**: mirovanje dok se ne pozove @ref start_pulse_sequence().
 *
 * ### Bezbednosne napomene
 * - Anoda i katoda **nikada** ne treba da budu istovremeno aktivne (shoot‑through). Implementacija
 *   obezbeđuje međufaze sa oba pina na 0 pre prelaza.
 * - Pre startovanja sekvence uveriti se da je **DC‑DC** napajanje omogućeno i stabilno.
 * - MUX patterni treba da budu usaglašeni sa stvarnim ožičenjem; pogrešan uzorak može premostiti
 *   neočekivane kanale.
 *
 * ### Konkurentnost / okruženje
 * - Modul koristi **k_work_delayable** (workqueue) – pozivi API‑ja su *thread‑context* bezbedni.
 * - @ref start_pulse_sequence i @ref stop_pulse_sequence su **idempotentni** (pozivi u toku imaju
 *   benigni efekat).
 *
 * @note Parametri kao što su `pulse_width`, `frequency` i MUX pattern niz definišu se u drugim
 *       modulima (npr. BLE NUS komandama i u `impulse.c`). Ovaj header izlaže samo javni API.
 */

#ifndef IMPULSE_H
#define IMPULSE_H

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/**
 * @def STIMULATION_PULSE_WIDTH_US
 * @brief Osnovna širina pojedinačne faze impulsa (µs).
 *
 * @details
 * Efektivna širina faze = `STIMULATION_PULSE_WIDTH_US * pulse_width`.
 * Vrednost `pulse_width` podešava se u BLE komandama (vidi `SW`).
 */
#define STIMULATION_PULSE_WIDTH_US 10

extern volatile uint16_t pair_amplitude_uA[8];
extern volatile uint32_t stim_duration_s;





/* === Devicetree aliasi (koriste se u .c za izvlačenje GPIO specifikacija) === */
#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)



/** @name GPIO specifikacije (popunjene u .c iz DT aliasa)
 *  @{
 */
/** GPIO pin specifikacija za katodu. */
extern const struct gpio_dt_spec pulse_cathode;
/** GPIO pin specifikacija za anodu. */
extern const struct gpio_dt_spec pulse_anode;
/** GPIO pin za uključivanje DC‑DC konvertora. */
extern const struct gpio_dt_spec dc_dc_en;
/** @} */




/**
 * @brief Globalna MUX konfiguracija koju koristi sekvencer impulsa.
 *
 * @details
 * `stim_mux_config` se inicijalizuje u `main.c` (npr. izbor SPI i GPIO uređaja,
 * LE/CLR pinova i broja kanala). Implementacija impulsa koristi ovu instancu
 * prilikom slanja pattern‑a u MUX.
 */
extern struct mux_config stim_mux_config;




/**
 * @brief Pretvara frekvenciju u period.
 *
 * @param frequency_hz Frekvencija u Hz. Ako je 0, vraća 0.
 * @return Period u mikrosekundama (µs).
 *
 * @note 1 s = 1 000 000 µs. Račun je celobrojni, tj. `1e6 / f`.
 */
uint32_t hz_to_us(uint32_t frequency_hz);











/**
 * @brief Startuje generisanje bifaznog stimulacionog impulsa.
 *
 * @details
 * - Ako sekvenca već traje, poziv nema efekta (idempotentno).
 * - Postavlja inicijalni pattern na MUX (npr. 0x0001),
 * - Resetuje brojač pulseva/pattern‑indeks,
 * - Aktivira *work* koji pokreće stroj stanja (anoda → katoda → pauza).
 *
 * @note Očekuje se da je @ref init_gpios već uspešno pozvan i da je DC‑DC
 *       napajanje uključeno.
 */
void start_pulse_sequence(void);


/**
 * @brief Zaustavlja generisanje impulsa i gasi izlaze.
 *
 * @details
 * - Otkaže odloženi work (ako je raspoređen),
 * - Resetuje stroj stanja u *IDLE*,
 * - Postavlja anodu i katodu na 0 (bezbedno stanje).
 * - Poziv je bezbedan i kada sekvenca već nije aktivna.
 */
void stop_pulse_sequence(void);





#endif /* IMPULSE_H */