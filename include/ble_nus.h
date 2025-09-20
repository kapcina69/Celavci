#ifndef BLE_NUS_H
#define BLE_NUS_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Global variables used for stimulation configuration. */
extern volatile uint8_t amplitude;     /**< Stimulation amplitude (1–30). */
extern uint8_t frequency;              /**< Stimulation frequency (1–40 Hz). */
extern uint8_t pulse_width;            /**< Pulse width in units (1–10). */
extern uint8_t temperature;            /**< Heating temperature (25–42 °C). */
extern uint8_t stim_state;             /**< Stimulation state (0–3). */
extern uint8_t RCE;
/* NOVO: Impulse integration flag */
extern bool impulse_initialized;
extern uint8_t new_frequency;  /**< Flag indicating if frequency was updated. */

extern bool stimulation_running;  /**< Indicates if stimulation is active. */

/**
 * @brief Callback called when BLE stack is ready.
 *
 * @param err Error code (0 if successful).
 */
void bt_ready(int err);

/**
 * @brief Callback triggered when a BLE connection is established.
 *
 * @param conn Pointer to the active BLE connection.
 * @param err Error code during connection (0 if successful).
 */
void connected(struct bt_conn *conn, uint8_t err);

/**
 * @brief Callback triggered when a BLE connection is terminated.
 *
 * @param conn Pointer to the BLE connection.
 * @param reason Reason for disconnection.
 */
void disconnected(struct bt_conn *conn, uint8_t reason);

/**
 * @brief Initializes the BLE stack and the Nordic UART Service (NUS).
 *
 * Initializes Bluetooth, registers all relevant callbacks, and starts the NUS service.
 *
 * @return 0 if successful, otherwise a negative error code.
 */
int ble_nus_init(void);

/**
 * @brief Sends a response message over the Nordic UART Service (NUS).
 *
 * @param msg Null-terminated string message to send.
 */
void send_response(const char *msg);



static void process_command(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_NUS_H
