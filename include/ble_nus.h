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

/**
 * @brief Parses and processes an ASCII command string received over BLE.
 *
 * Accepts commands in the format "XX;NN", where:
 * - "XX" is a two-character command identifier (e.g., "ST", "SA", "SF", "SW", "HT"),
 * - ";" is the separator,
 * - "NN" is a numeric value in decimal (1–3 digits).
 *
 * Supported commands:
 * - "ST": Sets `stim_state`   → values: 0–3
 * - "SA": Sets `amplitude`    → values: 1–30
 * - "SF": Sets `frequency`    → values: 1–40 Hz
 * - "SW": Sets `pulse_width`  → values: 1–10
 * - "HT": Sets `temperature`  → values: 25–42 °C
 *
 * Sends `OK_MSG` on success, or `ERR_MSG` if the command or value is invalid.
 *
 * @param data Pointer to the received byte array.
 * @param len Length of the received data.
 */
static void process_command(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_NUS_H
