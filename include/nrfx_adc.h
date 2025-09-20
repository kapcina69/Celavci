



#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>



/* ====== CONFIGURATION CONSTANTS ====== */

/**
 * @def SAADC_DEFAULT_THRESHOLD_MV
 * @brief Default contact detection threshold in millivolts
 */
#define SAADC_DEFAULT_THRESHOLD_MV    100u

/**
 * @def SAADC_DEFAULT_HYSTERESIS_MV  
 * @brief Default hysteresis around threshold in millivolts
 */
#define SAADC_DEFAULT_HYSTERESIS_MV   10u

/**
 * @def SAADC_MAX_THRESHOLD_MV
 * @brief Maximum configurable threshold (limited by 600mV reference)
 */
#define SAADC_MAX_THRESHOLD_MV        600u

/**
 * @def SAADC_BURST_LENGTH
 * @brief Number of pulses per burst group
 */
#define SAADC_BURST_LENGTH            8u

/**
 * @def SAADC_FAST_TIMEOUT_MS
 * @brief Ultra-fast timeout for single measurements
 */
#define SAADC_FAST_TIMEOUT_MS         2u

/* ====== CALLBACK TYPES ====== */

/**
 * @brief Callback function type for SAADC measurements
 * 
 * @param sample Raw 12-bit ADC sample value (0-4095)
 * 
 * @note This callback is called from ISR context, keep it minimal!
 */
typedef void (*saadc_cb_t)(int16_t sample);

/* ====== ERROR CODES ====== */

/**
 * @brief SAADC-specific error codes
 */
typedef enum {
    SAADC_ERROR_NONE = 0,           /**< No error */
    SAADC_ERROR_NOT_INITIALIZED,    /**< Module not initialized */
    SAADC_ERROR_HARDWARE_FAULT,     /**< Hardware malfunction */
    SAADC_ERROR_TIMEOUT,            /**< Measurement timeout */
    SAADC_ERROR_BUSY,               /**< Previous measurement still active */
    SAADC_ERROR_INVALID_PARAM,      /**< Invalid parameter value */
    SAADC_ERROR_CALIBRATION_FAILED, /**< ADC calibration error */
    SAADC_ERROR_GPPI_ALLOCATION,    /**< (D)PPI channel allocation failed */
    SAADC_ERROR_RECOVERY_FAILED     /**< Error recovery unsuccessful */
} saadc_error_t;

/* ====== DIAGNOSTIC STRUCTURES ====== */

/**
 * @brief Comprehensive SAADC diagnostic information
 * 
 * @details
 * This structure provides complete visibility into the SAADC module state
 * for debugging, performance analysis, and system health monitoring.
 */
typedef struct {
    /* === Initialization State === */
    bool initialized;               /**< Module initialization status */
    bool hardware_ready;            /**< Hardware availability status */
    bool gppi_allocated;            /**< (D)PPI channel allocation status */
    
    /* === Runtime State === */
    bool measurement_active;        /**< Active measurement in progress */
    bool sample_ready;              /**< New sample available */
    uint32_t error_count;           /**< Current error streak counter */
    
    /* === Burst Tracking === */
    uint32_t burst_index;           /**< Current position in burst (0-7) */
    uint32_t burst_mask;            /**< Current burst accumulation mask */
    bool burst_data_valid;          /**< Last complete burst available */
    uint8_t last_burst_result;      /**< Last complete burst pattern */
    
    /* === Contact Detection === */
    bool contact_detected;          /**< Current contact state */
    uint16_t threshold_hi_code;     /**< High threshold (ADC codes) */
    uint16_t threshold_lo_code;     /**< Low threshold (ADC codes) */
    uint16_t last_sample_raw;       /**< Last raw ADC sample */
    
    /* === Performance Metrics === */
    uint32_t total_measurements;    /**< Total measurements since init */
    uint32_t successful_measurements; /**< Successful measurement count */
    uint32_t failed_measurements;   /**< Failed measurement count */
    uint32_t timeout_events;        /**< Timeout occurrence count */
    uint32_t recovery_events;       /**< Recovery procedure count */
    uint32_t contact_events;        /**< Contact detection events */
    uint32_t max_conversion_time_us; /**< Peak conversion time */
    
    /* === Hardware Status === */
    uint8_t gain_numerator;         /**< Current gain setting (numerator) */
    uint8_t gain_denominator;       /**< Current gain setting (denominator) */
    uint8_t gppi_channel;           /**< Allocated (D)PPI channel number */
    bool irq_connected;             /**< IRQ handler connection status */
} saadc_diagnostics_t;

/**
 * @brief Lightweight performance statistics
 * 
 * @details
 * Optimized structure for frequent performance monitoring without
 * the overhead of full diagnostics.
 */
typedef struct {
    uint32_t successful_measurements; /**< Successful conversions */
    uint32_t failed_measurements;     /**< Failed conversions */
    uint32_t timeout_events;          /**< Timeout occurrences */
    uint32_t recovery_events;         /**< Recovery procedures */
    uint32_t contact_events;          /**< Contact detections */
    uint32_t max_conversion_time_us;  /**< Peak timing */
} saadc_performance_t;

/* ====== CORE API FUNCTIONS ====== */

/**
 * @brief Initialize SAADC with standard configuration
 * 
 * @details
 * Sets up SAADC hardware with optimized settings for fast measurements:
 * - Single-ended input on configured pin
 * - 12-bit resolution
 * - Minimal acquisition time (3µs)
 * - (D)PPI chain for zero-latency triggering
 * - Default threshold: 100mV ± 10mV
 * 
 * @retval 0 Success
 * @retval -ENODEV Hardware not available
 * @retval -EIO Hardware initialization failed
 * @retval -ENOMEM (D)PPI channel allocation failed
 * 
 * @note This function can be called multiple times (idempotent)
 */
int saadc_ppi_oneshot_init(void);

/**
 * @brief Initialize SAADC optimized for impulse module integration
 * 
 * @details
 * Enhanced initialization with additional optimizations and built-in
 * self-test for impulse module integration:
 * - All standard optimizations
 * - Automatic self-test validation
 * - Error reporting integration
 * - Performance monitoring setup
 * 
 * @retval 0 Success
 * @retval -ENODEV Hardware not available  
 * @retval -EIO Hardware initialization failed
 * @retval -ETIME Self-test timeout
 * @retval -ENODATA Self-test data validation failed
 * 
 * @note Recommended for impulse module integration
 */
int saadc_fast_init_for_impulse(void);

/**
 * @brief Trigger ultra-fast single measurement
 * 
 * @details
 * **TIMING CRITICAL FUNCTION**
 * 
 * This function is designed to complete within the cathode phase duration
 * which can be as short as 1µs. The measurement is triggered via zero-latency
 * (D)PPI chain and completes asynchronously.
 * 
 * **Execution Profile:**
 * - Function overhead: <1µs
 * - Hardware trigger: <1µs  
 * - Total synchronous time: <2µs
 * - Conversion completes asynchronously in ~2µs
 * 
 * **Usage Pattern:**
 * ```c
 * set_cathode(1);               // Start cathode phase
 * saadc_trigger_once_ppi();     // <2µs trigger
 * k_busy_wait(cathode_width);   // Wait for cathode phase
 * set_cathode(0);               // End cathode phase
 * // Result available via callback or saadc_get_last()
 * ```
 * 
 * @retval 0 Measurement triggered successfully
 * @retval -EBUSY Previous measurement still active
 * @retval -ENODEV Module not initialized
 * @retval -EIO Hardware error
 * 
 * @warning Call only during cathode phase when measurement is valid!
 * @note Thread-safe, can be called from any context including ISR
 */
int saadc_trigger_once_ppi(void);

/* ====== DATA RETRIEVAL API ====== */

/**
 * @brief Get last completed measurement result
 * 
 * @param[out] out Pointer to store raw ADC sample (can be NULL)
 * 
 * @retval true New sample available
 * @retval false No new sample since last call
 * 
 * @note Thread-safe, non-blocking
 */
bool saadc_get_last(int16_t *out);

/**
 * @brief Get last completed burst pattern
 * 
 * @param[out] mask_out Pointer to store 8-bit burst mask (can be NULL)
 * 
 * @retval true Complete burst data available
 * @retval false No complete burst since last call
 * 
 * @details
 * Each bit in the mask represents the contact state for one pulse:
 * - Bit 0: First pulse in burst
 * - Bit 7: Last pulse in burst  
 * - 1 = Contact detected (above threshold)
 * - 0 = No contact (below threshold)
 * 
 * @note Thread-safe, atomic operation
 */
bool saadc_get_last_burst(uint8_t *mask_out);

/**
 * @brief Get current progress within burst
 * 
 * @return Current pulse index within burst (0-7)
 * 
 * @note Thread-safe, useful for debugging burst scheduling
 */
uint8_t saadc_get_burst_progress(void);

/**
 * @brief Get current contact detection state
 * 
 * @retval true Contact currently detected (above threshold)
 * @retval false No contact (below threshold)
 * 
 * @note Thread-safe, real-time status
 */
bool saadc_get_contact_state(void);

/**
 * @brief Get total measurement count since initialization
 * 
 * @return Total number of completed measurements
 * 
 * @note Thread-safe, useful for usage statistics
 */
uint32_t saadc_get_total_measurements(void);

/* ====== CONFIGURATION API ====== */

/**
 * @brief Set contact detection threshold dynamically
 * 
 * @param threshold_mv Threshold voltage in millivolts (1-600)
 * @param hysteresis_mv Hysteresis around threshold (0-threshold_mv)
 * 
 * @retval 0 Threshold updated successfully
 * @retval -ENODEV Module not initialized
 * @retval -EINVAL Invalid parameter values
 * 
 * @details
 * Updates threshold values without requiring reinitialization:
 * - High threshold = threshold_mv + hysteresis_mv
 * - Low threshold = threshold_mv - hysteresis_mv
 * - Changes take effect immediately
 * 
 * @note Thread-safe, can be called during operation
 */
int saadc_set_threshold(uint16_t threshold_mv, uint16_t hysteresis_mv);

/**
 * @brief Get current threshold configuration
 * 
 * @param[out] threshold_mv Current threshold in millivolts
 * @param[out] hysteresis_mv Current hysteresis in millivolts
 * 
 * @retval 0 Values retrieved successfully
 * @retval -EINVAL NULL pointer parameter
 * 
 * @note Thread-safe
 */
int saadc_get_threshold(uint16_t *threshold_mv, uint16_t *hysteresis_mv);

/**
 * @brief Register callback for measurement notifications
 * 
 * @param cb Callback function (NULL to disable)
 * 
 * @details
 * Callback is invoked from ISR context upon measurement completion.
 * Keep callback implementation minimal to avoid affecting timing.
 * 
 * @warning Callback runs in ISR context - minimize processing!
 * @note Thread-safe
 */
void saadc_set_callback(saadc_cb_t cb);

/* ====== PERFORMANCE MONITORING API ====== */

/**
 * @brief Get comprehensive performance statistics
 * 
 * @param[out] successful Successful measurements (can be NULL)
 * @param[out] failed Failed measurements (can be NULL)
 * @param[out] timeouts Timeout events (can be NULL)
 * @param[out] recoveries Recovery procedures (can be NULL)
 * @param[out] contacts Contact detection events (can be NULL)
 * @param[out] max_time_us Peak conversion time (can be NULL)
 * 
 * @note Thread-safe, all parameters optional
 */
void saadc_get_performance_stats(uint32_t *successful, uint32_t *failed, 
                                uint32_t *timeouts, uint32_t *recoveries,
                                uint32_t *contacts, uint32_t *max_time_us);

/**
 * @brief Reset all performance counters
 * 
 * @details
 * Resets all performance counters to zero while preserving
 * system configuration and state. Useful for session-based
 * performance analysis.
 * 
 * @note Thread-safe
 */
void saadc_reset_performance_stats(void);

/* ====== ERROR HANDLING & RECOVERY API ====== */

/**
 * @brief Force manual error recovery
 * 
 * @retval 0 Recovery initiated successfully
 * @retval -ENODEV Module not initialized
 * 
 * @details
 * Triggers immediate error recovery sequence:
 * - Abort any ongoing operations
 * - Reset hardware state
 * - Clear error counters
 * - Restore optimal configuration
 * 
 * @note Thread-safe, useful for debugging and error scenarios
 */
int saadc_force_recovery(void);

/**
 * @brief Get current error state
 * 
 * @return Current error streak count
 * 
 * @details
 * Returns the number of consecutive errors since last successful
 * operation. Automatic recovery is triggered when this exceeds
 * the configured threshold.
 * 
 * @note Thread-safe
 */
uint32_t saadc_get_error_count(void);

/* ====== SYSTEM LIFECYCLE API ====== */

/**
 * @brief Graceful system shutdown
 * 
 * @details
 * Performs complete module shutdown:
 * - Stop all ongoing operations
 * - Cancel scheduled maintenance
 * - Release (D)PPI resources
 * - Uninitialize hardware
 * - Log final statistics
 * 
 * @note Call before system shutdown or module unloading
 */
void saadc_shutdown(void);

/**
 * @brief Check if module is initialized and ready
 * 
 * @retval true Module ready for operation
 * @retval false Module not initialized or error state
 * 
 * @note Thread-safe, lightweight status check
 */
bool saadc_is_ready(void);



