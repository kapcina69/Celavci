/**
 * @file impulse.h
 * @brief Generisanje bifaznih (bipolarnih) stimulacionih impulsa preko GPIO i MUX.
 *
 * @details
 * Ovaj modul implementira **napredni stroj stanja** (state machine) koji u pravilnom redosledu
 * upravlja izlazima *anode* i *katode* (GPIO) i istovremeno sekvencira uzorke (pattern)
 * na analognom multiplekseru (MUX) radi adresiranja kanala stimulacije.
 *
 * ### NOVA POBOLJŠANJA U VERZIJI 2.0
 * - **Thread Pool Pattern**: Eliminiše memory leaks, thread se kreira jednom
 * - **Thread-Safe Design**: Atomske operacije, race condition protection
 * - **Robusan Error Handling**: Centralizovano upravljanje greškama sa auto-recovery
 * - **Hardware Watchdog**: Automatska detekcija i oporavak od zaglavljenih stanja
 * - **Performance Monitoring**: Real-time statistike za debugging i optimizaciju
 * - **Bezbedni Timing**: Overflow protection i sanity checks
 * - **Production Ready**: Sve identifikovane probleme rešene
 *
 * ### Pregled funkcionalnosti
 * - Inicijalizacija izlaznih pinova: **anoda**, **katoda**, **DC-DC enable**;
 * - Generisanje **bifaznog** impulsa: faza *anode ON* → faza *katode ON* → pauza;
 * - Nakon svakog kompletnog impulsa, **MUX** dobija sledeći pattern (2 bajta) da bi se ciklično
 *   izabrali naredni kanali (npr. 0x01, 0x02, 0x04, ... 0x80);
 * - Period ponavljanja sekvence zavisi od zadate **frekvencije** (podešava se preko BLE komandnog sloja).
 * - **Thread-safe komunikacija** između BLE i impulse thread-a.
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
 * - **IMPULSE_IDLE**: sistem u mirovanju, spreman za pokretanje
 * - **IMPULSE_RUNNING**: aktivna stimulacija u toku
 * - **IMPULSE_STOPPING**: graceful shutdown u toku
 *
 * ### Bezbednosne napomene
 * - Anoda i katoda **nikada** ne treba da budu istovremeno aktivne (shoot‑through). Implementacija
 *   obezbeđuje međufaze sa oba pina na 0 pre prelaza.
 * - Pre startovanja sekvence uveriti se da je **DC‑DC** napajanje omogućeno i stabilno.
 * - MUX patterni treba da budu usaglašeni sa stvarnim ožičenjem; pogrešan uzorak može premostiti
 *   neočekivane kanale.
 * - **Hardware watchdog** automatski detektuje i oporavlja sistem od grešaka.
 *
 * ### Konkurentnost / okruženje
 * - Modul koristi **k_work_delayable** (workqueue) – pozivi API‑ja su *thread‑context* bezbedni.
 * - @ref start_pulse_sequence i @ref stop_pulse_sequence su **idempotentni** i **thread-safe**.
 * - Koriste se **atomske operacije** za thread-safe komunikaciju.
 * - **Thread pool pattern** eliminiše memory leaks i poboljšava performance.
 *
 * ### Performance karakteristike
 * - **Timing precision**: ±1µs (k_busy_wait)
 * - **Thread safety**: Atomske operacije, bez race conditions
 * - **Error recovery**: Automatski oporavak od hardware grešaka
 * - **Memory efficiency**: Nema memory leaks, stack reuse
 * - **Real-time performance**: High-priority thread (PRIO 6)
 * - **Hardware watchdog**: 1s timeout sa auto-recovery
 *
 * @note Parametri kao što su `pulse_width`, `frequency` i MUX pattern niz definišu se u drugim
 *       modulima (npr. BLE NUS komandama i u `impulse.c`). Ovaj header izlaže javni API.
 *
 * @author Celavci Team
 * @version 2.0
 * @date 2025
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
 * Vrednost `pulse_width` podešava se u BLE komandama (vidi `PW`).
 */
#define STIMULATION_PULSE_WIDTH_US 1 // Optimizovano za aplikaciju

/* === API KONSTANTE === */

/**
 * @def MAX_PATTERNS
 * @brief Maksimalan broj pattern-a koji se mogu učitati u memoriju
 */
#define MAX_PATTERNS 16

/**
 * @def PATTERN_LEN
 * @brief Broj kanala po pattern-u (8 channel pairs)
 */
#define PATTERN_LEN 8

/**
 * @def MAX_AMPLITUDE_UA
 * @brief Maksimalna amplitude po kanalu (µA)
 */
#define MAX_AMPLITUDE_UA 2000

/**
 * @def MIN_AMPLITUDE_UA
 * @brief Minimalna amplitude po kanalu (µA)
 */
#define MIN_AMPLITUDE_UA 5

/* === GLOBALNE VARIJABLE === */

/**
 * @brief Per-channel amplitude konfiguracija (µA)
 * 
 * @details Array od 8 elemenata koji definiše amplitude za svaki kanal par.
 * Vrednosti su u mikroamperima (µA) i ograničene su na MIN_AMPLITUDE_UA do MAX_AMPLITUDE_UA.
 * Thread-safe pristup kroz BLE komande.
 */
extern volatile uint16_t pair_amplitude_uA[8];

/**
 * @brief Maksimalno trajanje stimulacione sesije (sekunde)
 * 
 * @details Timer automatski zaustavlja stimulaciju nakon ovog vremena.
 * Default: 1800s (30 minuta). Thread-safe.
 */
extern volatile uint32_t stim_duration_s;

/**
 * @brief Pattern storage - 2D array pattern definicija
 * 
 * @details
 * - pulse_patterns[pattern_idx][channel_idx] = channel_mask
 * - pattern_idx: 0 do (MAX_PATTERNS-1) 
 * - channel_idx: 0 do (PATTERN_LEN-1)
 * - channel_mask: 16-bit vrednost za MUX kontrolu
 */
extern uint16_t pulse_patterns[MAX_PATTERNS][PATTERN_LEN];

/**
 * @brief Broj trenutno učitanih pattern-a
 * 
 * @details Thread-safe pristup. Maximum = MAX_PATTERNS.
 */
extern volatile size_t patterns_count;

/**
 * @brief Indeks trenutno aktivnog pattern-a
 * 
 * @details 
 * Thread-safe pristup kroz BLE SP komandu.
 * Mora biti < patterns_count, inače se koristi pattern 0.
 */
extern volatile size_t number_patter;

/* === DEVICETREE ALIASI === */
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

#ifdef __cplusplus
extern "C" {
#endif

/* === CORE API FUNCTIONS === */

/**
 * @brief Inicijalizuje impulse subsystem (NOVO u v2.0)
 * 
 * @details
 * - Kreira thread pool (thread se pravi jednom i nikad ne briše)
 * - Inicijalizuje semafore, timers, work queue
 * - Postavlja početno stanje sistema
 * - **MORA SE POZVATI JEDNOM iz main() pre korišćenja bilo koje druge funkcije**
 * 
 * @retval 0 Uspešna inicijalizacija
 * @retval -ENODEV Hardware greška (GPIO/SAADC)
 * @retval -ENOMEM Memory allocation greška
 * @retval -ETIME Thread startup timeout
 * 
 * @note Thread-safe, može se pozvati više puta (idempotentno)
 */
int impulse_init(void);

/**
 * @brief Startuje generisanje bifaznog stimulacionog impulsa.
 *
 * @details
 * - **Thread-safe**: Koristi atomske operacije za state management
 * - Ako sekvenca već traje, poziv nema efekta (idempotentno).
 * - Postavlja inicijalni pattern na MUX (iz aktivnog pattern-a),
 * - Resetuje brojač pulseva/pattern‑indeks,
 * - Signalizira thread-u da počne stimulaciju
 * - Aktivira session timer (auto-stop nakon stim_duration_s)
 * - Uključuje DC-DC napajanje
 *
 * @note Očekuje se da je @ref impulse_init već uspešno pozvan.
 * 
 * @warning Ne blokira - vraća se odmah. Stimulacija se izvršava u background thread-u.
 */
void start_pulse_sequence(void);

/**
 * @brief Zaustavlja generisanje impulsa i gasi izlaze.
 *
 * @details
 * - **Thread-safe**: Koristi atomske operacije
 * - Signalizira thread-u da prestane sa radom
 * - Otkaže session timer
 * - Postavlja anodu i katodu na 0 (bezbedno stanje)
 * - Isključuje DC-DC napajanje
 * - Poziv je bezbedan i kada sekvenca već nije aktivna (idempotentno)
 * - Generiše BLE response ("STOPPED")
 */
void stop_pulse_sequence(void);

/**
 * @brief Notifikuje impulse thread o promeni frekvencije (NOVO u v2.0)
 * 
 * @details
 * Thread-safe način da se signalizira promjena frekvencije.
 * Impulse thread će resetovati burst scheduling na sledeći ciklus.
 * 
 * @note Poziva se automatski iz BLE NUS kada se primi SF komanda.
 */
void notify_frequency_changed(void);

/* === UTILITY FUNCTIONS === */

/**
 * @brief Pretvara frekvenciju u period sa overflow protection (POBOLJŠANO u v2.0)
 *
 * @param frequency_hz Frekvencija u Hz. Ako je 0, vraća 0.
 * @return Period u mikrosekundama (µs).
 *
 * @details 1 s = 1 000 000 µs. Račun je celobrojni, tj. `1e6 / f`.
 * **NOVO**: Dodata overflow protection za frekvencije > 1MHz.
 */
uint32_t hz_to_us(uint32_t frequency_hz);

/* === CONTACT DETECTION API === */

/**
 * @brief Šalje poslednji burst rezultat preko BLE (thread-safe)
 * 
 * @details
 * Čita poslednju kompletnu burst sekvencu (8 impulsa) iz SAADC modula
 * i šalje rezultat kao ">RCE;mask<" preko BLE NUS servisa.
 * 
 * @note Thread-safe, može se pozvati iz bilo kog konteksta.
 */
void report_last_burst(void);

/* === PERFORMANCE MONITORING API (NOVO u v2.0) === */

/**
 * @brief Vraća trenutne performance statistike
 * 
 * @param[out] pulses Total broj generisanih impulsa (može biti NULL)
 * @param[out] errors Total broj grešaka (MUX + DAC) (može biti NULL)  
 * @param[out] recoveries Broj izvršenih automatskih recovery-ja (može biti NULL)
 * 
 * @details
 * Thread-safe pristup performance counter-ima.
 * Korisno za debugging, monitoring kvaliteta servisa i analizu.
 * 
 * @note Counters se resetuju pri svakom start_pulse_sequence().
 */
void get_performance_stats(uint32_t *pulses, uint32_t *errors, uint32_t *recoveries);

/* === ADVANCED API (Opciono za napredne korisnike) === */

/**
 * @brief Vraća trenutno stanje impulse state machine-a
 * 
 * @retval 0 IMPULSE_IDLE - sistem u mirovanju
 * @retval 1 IMPULSE_RUNNING - aktivna stimulacija  
 * @retval 2 IMPULSE_STOPPING - graceful shutdown u toku
 * 
 * @note Thread-safe, atomska operacija.
 */
int get_impulse_state(void);

/**
 * @brief Vraća broj impulsa generisanih u trenutnoj sesiji
 * 
 * @return Broj impulsa (atomska operacija)
 * 
 * @note Thread-safe, reset na svaki start_pulse_sequence().
 */
uint32_t get_pulse_counter(void);

/* === ERROR HANDLING TYPES === */

/**
 * @brief Error códovi specifični za impulse modul
 */
typedef enum {
    IMPULSE_ERROR_NONE = 0,           /**< Nema grešaka */
    IMPULSE_ERROR_NOT_INITIALIZED,    /**< impulse_init() nije pozvan */
    IMPULSE_ERROR_HARDWARE_FAILURE,   /**< GPIO/SAADC greška */
    IMPULSE_ERROR_MUX_FAILURE,        /**< SPI MUX komunikacija neuspešna */
    IMPULSE_ERROR_DAC_FAILURE,        /**< I2C DAC komunikacija neuspešna */
    IMPULSE_ERROR_TIMING_VIOLATION,   /**< Timing constraint prekršen */
    IMPULSE_ERROR_PATTERN_INVALID,    /**< Pattern indeks van opsega */
    IMPULSE_ERROR_AMPLITUDE_INVALID,  /**< Amplitude van dozvoljenog opsega */
    IMPULSE_ERROR_WATCHDOG_TIMEOUT    /**< Hardware watchdog timeout */
} impulse_error_t;

/**
 * @brief Callback tip za error notifications (opciono)
 * 
 * @param error_code Tip greške
 * @param context Dodatne informacije o grešci (može biti NULL)
 */
typedef void (*impulse_error_callback_t)(impulse_error_t error_code, const char *context);

/**
 * @brief Registruje callback za error notifications (NOVO u v2.0)
 * 
 * @param callback Funkcija koja će biti pozvana pri grešci (NULL = disable)
 * 
 * @details
 * Omogućava aplikaciji da reaguje na greške u real-time.
 * Thread-safe, callback se poziva iz work queue konteksta.
 */
void impulse_set_error_callback(impulse_error_callback_t callback);

/* === DIAGNOSTICS API (NOVO u v2.0) === */

/**
 * @brief Vraća detaljne diagnostike sistema
 * 
 * @param[out] info Struktura sa diagnostic informacijama
 * @retval 0 Uspeh
 * @retval -EINVAL info je NULL
 * 
 * @note Thread-safe
 */
typedef struct {
    uint32_t total_pulses;           /**< Ukupno impulsa od boot-a */
    uint32_t session_pulses;         /**< Impulsi u trenutnoj sesiji */
    uint32_t mux_errors;             /**< MUX greške */
    uint32_t dac_errors;             /**< DAC greške */
    uint32_t timing_violations;      /**< Timing prekršaji */
    uint32_t recoveries;             /**< Automatski recovery-ji */
    uint32_t uptime_ms;              /**< Vreme rada thread-a (ms) */
    uint8_t  current_pattern;        /**< Aktivni pattern indeks */
    uint8_t  current_channel;        /**< Trenutni kanal (0-7) */
    uint16_t last_amplitude_ua;      /**< Poslednja DAC amplitude (µA) */
    bool     hardware_ok;            /**< Overall hardware status */
} impulse_diagnostics_t;

int get_impulse_diagnostics(impulse_diagnostics_t *info);

#ifdef __cplusplus
}
#endif

#endif /* IMPULSE_H */