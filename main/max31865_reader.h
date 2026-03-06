#ifndef PT100_LOGGER_MAX31865_READER_H_
#define PT100_LOGGER_MAX31865_READER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    kMax31865ConversionTablePt100 = 0,
    kMax31865ConversionCvdIterative = 1,
  } max31865_conversion_t;

  typedef struct
  {
    uint16_t adc_code;
    double resistance_ohm;
    double temperature_c;
    uint8_t fault_status;
    bool fault_present;
  } max31865_sample_t;

  typedef enum
  {
    kMax31865OneShotIdle = 0,
    kMax31865OneShotBiasSettling,
    kMax31865OneShotConverting,
  } max31865_one_shot_phase_t;

  typedef struct
  {
    max31865_one_shot_phase_t phase;
    uint8_t base_config;
    int64_t next_action_deadline_us;
    int64_t hard_timeout_deadline_us;
    bool conversion_started;
  } max31865_one_shot_state_t;

  typedef struct
  {
    int requested_samples;
    int valid_samples;
    int faulted_samples;
    double min_temp_c;
    double max_temp_c;
    double min_resistance_ohm;
    double max_resistance_ohm;
    double stddev_temp_c;
  } max31865_sampling_stats_t;

  typedef struct
  {
    spi_device_handle_t spi_device;
    double rtd_nominal_ohm; // e.g. 100.0 for PT100
    double rref_ohm;        // e.g. 430.0 on common breakout boards
    uint8_t wires;          // 2, 3, or 4
    uint8_t filter_hz;      // 50 or 60
    uint32_t bias_settle_ms;
    int rdy_gpio;
    bool rdy_enabled;
    uint8_t rdy_ready_level;
    max31865_conversion_t conversion;
    bool pulsed_bias;
    bool is_initialized;
    double ema_temp_c;
    double ema_resistance_ohm;
    bool ema_valid;

    // DMA-safe staging buffers (internal RAM) used to avoid per-transaction
    // bounce-buffer allocations when the calling task stack lives in PSRAM.
    // These are allocated during Max31865ReaderInit() and reused for all
    // transfers.
    uint8_t* dma_tx_buf;
    uint8_t* dma_rx_buf;
    size_t dma_buf_len;

    // Optional SPI bus lock callbacks for shared bus coordination.
    bool (*spi_bus_lock)(void* context, TickType_t timeout_ticks);
    void (*spi_bus_unlock)(void* context);
    void* spi_bus_lock_context;
    TickType_t spi_bus_lock_timeout_ticks;
  } max31865_reader_t;

  // Initialize MAX31865 on an already-initialized SPI bus.
/**
 * @brief Execute Max31865ReaderInit.
 * @param reader Parameter reader.
 * @param host Parameter host.
 * @param cs_gpio Parameter cs_gpio.
 * @return Return the function result.
 */
  esp_err_t Max31865ReaderInit(max31865_reader_t* reader,
                               spi_host_device_t host,
                               int cs_gpio);
/**
 * @brief Execute Max31865ReaderDeinit.
 * @param reader Parameter reader.
 * @return Return the function result.
 */
  esp_err_t Max31865ReaderDeinit(max31865_reader_t* reader);

  // Register helpers.
/**
 * @brief Execute Max31865ReadReg.
 * @param reader Parameter reader.
 * @param reg Parameter reg.
 * @param value_out Parameter value_out.
 * @return Return the function result.
 */
  esp_err_t Max31865ReadReg(max31865_reader_t* reader,
                            uint8_t reg,
                            uint8_t* value_out);
/**
 * @brief Execute Max31865ReadRegs.
 * @param reader Parameter reader.
 * @param reg Parameter reg.
 * @param data_out Parameter data_out.
 * @param len Parameter len.
 * @return Return the function result.
 */
  esp_err_t Max31865ReadRegs(max31865_reader_t* reader,
                             uint8_t reg,
                             uint8_t* data_out,
                             size_t len);
/**
 * @brief Execute Max31865WriteReg.
 * @param reader Parameter reader.
 * @param reg Parameter reg.
 * @param value Parameter value.
 * @return Return the function result.
 */
  esp_err_t Max31865WriteReg(max31865_reader_t* reader,
                             uint8_t reg,
                             uint8_t value);

  // Read one conversion using pulsed bias + one-shot mode.
/**
 * @brief Start a non-blocking one-shot conversion sequence.
 * @param reader Initialized MAX31865 reader instance.
 * @param state_out One-shot state storage owned by the caller.
 * @return ESP_OK when one-shot sequencing has started; error otherwise.
 *
 * Enables VBIAS, clears fault state, and initializes an asynchronous one-shot
 * state machine. This function does not block for bias settle or conversion.
 */
  esp_err_t Max31865StartOneShot(max31865_reader_t* reader,
                                 max31865_one_shot_state_t* state_out);

/**
 * @brief Advance and optionally finish a started one-shot conversion.
 * @param reader Initialized MAX31865 reader instance.
 * @param state In/out state previously initialized by Max31865StartOneShot().
 * @param sample_out Receives the completed sample when ESP_OK is returned.
 * @return ESP_OK when @p sample_out is valid, ESP_ERR_TIMEOUT while not ready,
 *         or another ESP_ERR_* value on I/O/configuration failure.
 *
 * This function progresses the one-shot state machine without blocking. The
 * caller should poll until ESP_OK or a terminal error is returned.
 */
  esp_err_t Max31865TryReadOneShot(max31865_reader_t* reader,
                                   max31865_one_shot_state_t* state,
                                   max31865_sample_t* sample_out);

/**
 * @brief Abort a started one-shot conversion and force the state idle.
 * @param reader Initialized MAX31865 reader instance.
 * @param state One-shot state to abort.
 * @return ESP_OK if abort cleanup succeeds; error if SPI/config write fails.
 *
 * This disables VBIAS using the tracked base configuration and resets the
 * caller-owned state machine so acquisition can be restarted safely.
 */
  esp_err_t Max31865AbortOneShot(max31865_reader_t* reader,
                                 max31865_one_shot_state_t* state);

/**
 * @brief Restore base configuration, disable VBIAS, and clear latched faults.
 * @param reader Initialized MAX31865 reader.
 * @return ESP_OK on success; error otherwise.
 */
  esp_err_t Max31865RecoverToBaseConfig(max31865_reader_t* reader);

/**
 * @brief Execute Max31865ReadOnce.
 * @param reader Parameter reader.
 * @param sample_out Parameter sample_out.
 * @return Return the function result.
 */
  esp_err_t Max31865ReadOnce(max31865_reader_t* reader,
                             max31865_sample_t* sample_out);

  // Read multiple samples and report the mean (faulted samples are skipped).
  // stats_out is optional; if provided stddev_temp_c will be populated for the
  // non-faulted samples.
/**
 * @brief Execute Max31865ReadAveraged.
 * @param reader Parameter reader.
 * @param sample_count Parameter sample_count.
 * @param sample_delay_ms Parameter sample_delay_ms.
 * @param averaged_out Parameter averaged_out.
 * @param stats_out Parameter stats_out.
 * @return Return the function result.
 */
  esp_err_t Max31865ReadAveraged(max31865_reader_t* reader,
                                 int sample_count,
                                 int sample_delay_ms,
                                 max31865_sample_t* averaged_out,
                                 max31865_sampling_stats_t* stats_out);

  // Update an EMA filter with the latest resistance. ema_temp_out may be NULL.
/**
 * @brief Execute Max31865ReadEmaUpdate.
 * @param reader Parameter reader.
 * @param alpha Parameter alpha.
 * @param sample_out Parameter sample_out.
 * @param ema_temp_out Parameter ema_temp_out.
 * @return Return the function result.
 */
  esp_err_t Max31865ReadEmaUpdate(max31865_reader_t* reader,
                                  double alpha,
                                  max31865_sample_t* sample_out,
                                  double* ema_temp_out);

/**
 * @brief Update MAX31865 EMA state using an already-acquired sample.
 * @param reader Initialized MAX31865 reader instance.
 * @param alpha EMA coefficient in (0, 1].
 * @param sample Previously acquired RTD sample.
 * @param ema_temp_out Optional EMA temperature output.
 * @return ESP_OK when EMA state is updated or sample is faulted;
 *         ESP_ERR_INVALID_ARG for invalid parameters.
 */
  esp_err_t Max31865ApplyEmaSample(max31865_reader_t* reader,
                                   double alpha,
                                   const max31865_sample_t* sample,
                                   double* ema_temp_out);

  // Legacy convenience wrapper returning floats.
/**
 * @brief Execute Max31865ReaderRead.
 * @param reader Parameter reader.
 * @param raw_temp_c Parameter raw_temp_c.
 * @param resistance_ohm Parameter resistance_ohm.
 * @return Return the function result.
 */
  esp_err_t Max31865ReaderRead(max31865_reader_t* reader,
                               float* raw_temp_c,
                               float* resistance_ohm);

  // Helpers for diagnostics.
/**
 * @brief Execute Max31865FormatFault.
 * @param fault_status Parameter fault_status.
 * @param out Parameter out.
 * @param out_len Parameter out_len.
 */
  void Max31865FormatFault(uint8_t fault_status, char* out, size_t out_len);
/**
 * @brief Execute Max31865AdcCodeToResistance.
 * @param adc_code Parameter adc_code.
 * @param rref_ohm Parameter rref_ohm.
 * @return Return the function result.
 */
  double Max31865AdcCodeToResistance(uint16_t adc_code, double rref_ohm);

/**
 * @brief Convert RTD resistance to temperature using the reader conversion mode.
 * @param reader Reader configuration containing conversion mode and r0.
 * @param resistance_ohm RTD resistance in ohms.
 * @return Temperature in Celsius.
 */
  double Max31865ResistanceToTemperature(const max31865_reader_t* reader,
                                         double resistance_ohm);

/**
 * @brief Convert temperature to PT100 resistance using CVD coefficients.
 * @param temperature_c Temperature in Celsius.
 * @param r0_ohm Nominal RTD resistance at 0C.
 * @return Resistance in ohms.
 */
  double Max31865TemperatureToResistanceCvd(double temperature_c,
                                            double r0_ohm);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MAX31865_READER_H_
