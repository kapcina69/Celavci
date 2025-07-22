#ifndef BLE_NUS_H
#define BLE_NUS_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#ifdef __cplusplus
extern "C" {
#endif


extern uint8_t amplitude;
extern uint8_t frequency;
extern uint8_t pulse_width;
extern uint8_t temperature;
extern uint8_t stim_state;


/**
 * @brief Callback koji se poziva kada je BLE spreman
 * @param err Greška pri pokretanju BLE (0 ako je uspešno)
 * @return void
 * 
 */
void bt_ready(int err);

/**
 * @brief Callback za događaj uspostavljene konekcije
 * 
 * @param conn Povezana BLE konekcija
 * @param err Greška pri uspostavljanju konekcije (0 ako je uspešno)
 */
void connected(struct bt_conn *conn, uint8_t err);
/**
 * @brief Callback za događaj prekida konekcije
 * @param conn Povezana BLE konekcija
 * @param reason Razlog prekida konekcije
 */
void disconnected(struct bt_conn *conn, uint8_t reason);

/**
 * @brief Inicijalizacija BLE i NUS servisa
 * Inicijalizuje Bluetooth, registruje callback-ove i pokreće NUS servis.
 * @return 0 ako je uspešno, ili greška kod neuspeha
 * 
 */
int ble_nus_init(void);


/**
 * @brief Pošalji odgovor preko NUS servisa
 * @param msg Poruka koju treba poslati
 */
static void send_response(const char *msg);


/**
 * @brief Obradjuje komandni niz u ASCII formatu i postavlja odgovarajuće parametre.
 *
 * Funkcija prima niz bajtova `data` koji sadrži komandni string u formatu "XX;NN", gde:
 *  - "XX" predstavlja dvočlanu komandu (npr. "ST", "SA", "SF", "SW", "HT"),
 *  - ";" je separator,
 *  - "NN" je numerička vrednost (1 do 3 cifre u decimalnom formatu).
 *
 * Na osnovu komande, funkcija postavlja sledeće globalne promenljive:
 *  - "ST": `stim_state` (0–3)
 *  - "SA": `amplitude` (1–30)
 *  - "SF": `frequency` (1–40)
 *  - "SW": `pulse_width` (1–10)
 *  - "HT": `temperature` (25–42)
 *
 * U slučaju validne komande i vrednosti, šalje se odgovor `OK_MSG`. 
 * U suprotnom, funkcija vraća `ERR_MSG`.
 *
 * @param data Ulazni niz bajtova koji sadrži komandu
 * @param len Dužina ulaznog niza
 */
static void process_command(const uint8_t *data, uint16_t len);



#ifdef __cplusplus
}
#endif

#endif // BLE_NUS_H
