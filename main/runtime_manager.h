#ifndef PT100_LOGGER_RUNTIME_MANAGER_H_
#define PT100_LOGGER_RUNTIME_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

#include "alerts/alert_manager.h"
#include "app_settings.h"
#include "data_port.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "fram_error_log.h"
#include "fram_i2c.h"
#include "fram_io.h"
#include "fram_log.h"
#include "i2c_bus.h"
#include "max31865_reader.h"
#include "mesh_transport.h"
#include "runtime_state.h"
#include "sd_logger.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct
  {
    app_settings_t* settings;
    fram_i2c_t* fram_i2c;
    fram_io_t* fram_io;
    fram_log_t* fram_log;
    fram_error_log_t* fram_error_log;
    sd_logger_t* sd_logger;
    max31865_reader_t* sensor;
    mesh_transport_t* mesh;
    time_sync_t* time_sync;
    // Calibration due-check runtime state (written by runtime_manager)
    bool* cal_due_check_suspended;
    bool* cal_overdue;
    bool* cal_time_stable;
    i2c_bus_t* i2c_bus;
    const char* node_id_string;
    alert_manager_t* alert_manager;
    esp_err_t (*flush_callback)(void* context);
    void* flush_context;
    bool* fram_full;
    uint32_t* export_dropped_count;
    uint32_t* export_write_fail_count;
    QueueHandle_t* export_outbox;
    QueueHandle_t* broker_outbox;
    uint32_t* export_drop_count;
    uint32_t* export_send_fail_count;
    uint32_t* broker_drop_count;
    uint32_t* broker_send_fail_count;
    bool* mqtt_client_connected;
  } app_runtime_t;

  typedef struct
  {
    int32_t flushed_records;
    int32_t remaining_records;
    int32_t flushed_bytes;
    int32_t duration_ms;
    esp_err_t result;
  } sd_drain_stats_t;

  typedef struct
  {
    uint32_t sample_id;
    int64_t timestamp_us;
    int32_t temp_mC;
    int32_t ohm_mohm;
    uint16_t fault;
    bool valid;
  } runtime_sensor_sample_t;

  /**
   * @brief Execute RuntimeManagerInit.
   * @return Return the function result.
   */
  esp_err_t RuntimeManagerInit(void);

  /**
   * @brief Execute RuntimeManagerInitMinimal.
   * @return Return the function result.
   */
  esp_err_t RuntimeManagerInitMinimal(void);

  /**
   * @brief Execute RuntimeManagerInitFull.
   * @return Return the function result.
   */
  esp_err_t RuntimeManagerInitFull(void);

  /**
   * @brief Execute RuntimeManagerRunSafeHoldIfNeeded.
   * @param state Parameter state.
   * @return Return the function result.
   */
  esp_err_t RuntimeManagerRunSafeHoldIfNeeded(runtime_state_t* state);

  /**
   * @brief Execute RuntimeGetRuntime.
   * @return Return the function result.
   */
  const app_runtime_t* RuntimeGetRuntime(void);

  /**
   * @brief Execute RuntimeGetEffectiveDisplayUnits.
   * @return Return the function result.
   */
  app_display_units_t RuntimeGetEffectiveDisplayUnits(void);

  /**
   * @brief Execute RuntimeGetCachedStatus.
   * @return Return the function result.
   */
  const runtime_cached_status_t* RuntimeGetCachedStatus(void);

  /**
   * @brief Execute RuntimeGetState.
   * @return Return the function result.
   */
  runtime_state_t* RuntimeGetState(void);

  /**
   * @brief Build PTLOG header metadata from current runtime state.
   * @param epoch_utc Header creation timestamp.
   * @param header_out Destination PTLOG header.
   * @param signature_out Optional signature output.
   * @return True on success.
   */
  bool RuntimeBuildPtlogHeader(int64_t epoch_utc,
                               ptlog_header_t* header_out,
                               uint32_t* signature_out);

  /**
   * @brief Execute RuntimeRebootAlertLatchIsPending.
   * @return Return the function result.
   */
  bool RuntimeRebootAlertLatchIsPending(void);

  /**
   * @brief Execute RuntimeRebootAlertLatchClearSticky.
   */
  void RuntimeRebootAlertLatchClearSticky(void);

  /**
   * @brief Execute RuntimeRebootAlertLatchCopy.
   * @param out Parameter out.
   */
  void RuntimeRebootAlertLatchCopy(runtime_reboot_alert_latch_t* out);

  /**
   * @brief Return the count of RTC-backed errlog entries pending flush.
   * @return Pending entry count.
   */
  uint16_t RuntimeRtcErrlogLatchPendingCount(void);

  /**
   * @brief Attempt to initialize the FRAM error log and flush pending latches.
   * @param state Parameter state.
   * @return Return the function result.
   */
  esp_err_t RuntimeMaybeInitFramErrorLog(runtime_state_t* state);

  /**
   * @brief Execute RuntimeI2cLock.
   * @param timeout_ticks Parameter timeout_ticks.
   * @return Return the function result.
   */
  bool RuntimeI2cLock(TickType_t timeout_ticks);

  /**
   * @brief Execute RuntimeI2cUnlock.
   */
  void RuntimeI2cUnlock(void);

  /**
   * @brief Execute RuntimeStart.
   * @return Return the function result.
   */
  esp_err_t RuntimeStart(void);

  /**
   * @brief Execute RuntimeStop.
   * @return Return the function result.
   */
  esp_err_t RuntimeStop(void);

  /**
   * @brief Execute RuntimeIsRunning.
   * @return Return the function result.
   */
  bool RuntimeIsRunning(void);

  /**
   * @brief Copy latest runtime-owned MAX31865 snapshot for console consumers.
   * @param out_sample Destination sample snapshot.
   * @return True when at least one sample has been published.
   */
  bool RuntimeCopyLatestSensorSample(runtime_sensor_sample_t* out_sample);

  /**
   * @brief Execute RuntimeApplyNetMode.
   * @param mode Parameter mode.
   * @return Return the function result.
   */
  esp_err_t RuntimeApplyNetMode(app_net_mode_t mode);

  /**
   * @brief Execute EnterRunMode.
   * @return Return the function result.
   */
  esp_err_t EnterRunMode(void);

  /**
   * @brief Execute EnterDiagMode.
   * @return Return the function result.
   */
  esp_err_t EnterDiagMode(void);

  typedef esp_err_t (*runtime_sd_op_fn_t)(app_runtime_t* runtime, void* ctx);

  /**
   * @brief Execute RuntimeWithTemporarySdMount.
   * @param op Parameter op.
   * @param ctx Parameter ctx.
   * @return Return the function result.
   */
  esp_err_t RuntimeWithTemporarySdMount(runtime_sd_op_fn_t op, void* ctx);

  /**
   * @brief Execute RuntimeSdFsLock.
   * @param state Parameter state.
   * @param timeout_ticks Parameter timeout_ticks.
   * @return Return the function result.
   */
  bool RuntimeSdFsLock(runtime_state_t* state, TickType_t timeout_ticks);

  /**
   * @brief Execute RuntimeSdFsUnlock.
   * @param state Parameter state.
   */
  void RuntimeSdFsUnlock(runtime_state_t* state);

  /**
   * @brief Execute RuntimeSdIoLock.
   * @param state Parameter state.
   * @param timeout_ticks Parameter timeout_ticks.
   * @return Return the function result.
   */
  bool RuntimeSdIoLock(runtime_state_t* state, TickType_t timeout_ticks);

  /**
   * @brief Execute RuntimeSdIoUnlock.
   * @param state Parameter state.
   */
  void RuntimeSdIoUnlock(runtime_state_t* state);

  /**
   * @brief Execute RuntimeSpiBusLockForSharedDevices.
   * @param state Parameter state.
   * @param timeout_ticks Parameter timeout_ticks.
   * @return Return the function result.
   */
  bool RuntimeSpiBusLockForSharedDevices(runtime_state_t* state,
                                         TickType_t timeout_ticks);

  /**
   * @brief Execute RuntimeSpiBusUnlockForSharedDevices.
   * @param state Parameter state.
   */
  void RuntimeSpiBusUnlockForSharedDevices(runtime_state_t* state);

  /**
   * @brief Dump current lock diagnostics to log output.
   * @param reason Reason string for the dump.
   */
  void RuntimeDumpLocksManual(const char* reason);

  /**
   * @brief Dump current I2C operation diagnostics to log output.
   * @param reason Reason string for the dump.
   */
  void RuntimeDumpI2cOpStateManual(const char* reason);

  /**
   * @brief Execute RuntimeDiagHeapCheck.
   * @param state Parameter state.
   * @param context Parameter context.
   * @param force Parameter force.
   * @return Return the function result.
   */
  bool RuntimeDiagHeapCheck(runtime_state_t* state,
                            const char* context,
                            bool force);

  /**
   * @brief Execute RuntimeRequestRunStart.
   */
  void RuntimeRequestRunStart(void);

  /**
   * @brief Execute RuntimeRequestRunStop.
   */
  void RuntimeRequestRunStop(void);

  /**
   * @brief Execute RuntimeNudgeWifiDirectTask.
   */
  void RuntimeNudgeWifiDirectTask(void);

  /**
   * @brief Execute RuntimeNudgeSensorTask.
   */
  void RuntimeNudgeSensorTask(void);

  /**
   * @brief Execute RuntimeSdUnmountNow.
   * @return Return the function result.
   */
  esp_err_t RuntimeSdUnmountNow(void);

  /**
   * @brief Execute RuntimeEnableDataStreaming.
   * @param enabled Parameter enabled.
   */
  void RuntimeEnableDataStreaming(bool enabled);

  /**
   * @brief Execute RuntimeIsDataStreamingEnabled.
   * @return Return the function result.
   */
  bool RuntimeIsDataStreamingEnabled(void);

  /**
   * @brief Execute RuntimeGetDataStreamBackend.
   * @return Return the function result.
   */
  DataPortBackend RuntimeGetDataStreamBackend(void);

  /**
   * @brief Execute RuntimeGetDataStreamInitError.
   * @return Return the function result.
   */
  esp_err_t RuntimeGetDataStreamInitError(void);

  /**
   * @brief Execute RuntimeIsI2cQuiesceActive.
   * @return Return the function result.
   */
  bool RuntimeIsI2cQuiesceActive(void);

  /**
   * @brief Execute RuntimeSetLogPolicyRun.
   */
  void RuntimeSetLogPolicyRun(void);

  /**
   * @brief Execute RuntimeSetLogPolicyDiag.
   */
  void RuntimeSetLogPolicyDiag(void);

  /**
   * @brief Execute RuntimeSetSdAppendFailureOnce.
   * @param enabled Parameter enabled.
   */
  void RuntimeSetSdAppendFailureOnce(bool enabled);

  /**
   * @brief Execute RuntimePrintStackMonitor.
   * @param headroom_bytes Parameter headroom_bytes.
   */
  void RuntimePrintStackMonitor(uint32_t headroom_bytes);

  /**
   * @brief Execute RuntimeSdIsDegraded.
   * @return Return the function result.
   */
  bool RuntimeSdIsDegraded(void);

  /**
   * @brief Execute RuntimeGetDisplaySpiHost.
   * @return Return the function result.
   */
  spi_host_device_t RuntimeGetDisplaySpiHost(void);

  /**
   * @brief Execute RuntimeGetDisplayMosiGpio.
   * @return Return the function result.
   */
  int RuntimeGetDisplayMosiGpio(void);

  /**
   * @brief Execute RuntimeGetDisplaySclkGpio.
   * @return Return the function result.
   */
  int RuntimeGetDisplaySclkGpio(void);

  /**
   * @brief Execute RuntimeGetDisplayCsGpio.
   * @return Return the function result.
   */
  int RuntimeGetDisplayCsGpio(void);

  /**
   * @brief Execute RuntimeSdFailCount.
   * @return Return the function result.
   */
  uint32_t RuntimeSdFailCount(void);

  /**
   * @brief Execute RuntimeSdBackoffUntilTicks.
   * @return Return the function result.
   */
  uint32_t RuntimeSdBackoffUntilTicks(void);

  /**
   * @brief Execute RuntimeGetSpiDeviceCount.
   * @return Return the function result.
   */
  uint32_t RuntimeGetSpiDeviceCount(void);

  /**
   * @brief Execute RuntimeAcknowledgeDisplayAttention.
   * @param item Parameter item.
   * @return Return the function result.
   */
  bool RuntimeAcknowledgeDisplayAttention(display_attention_item_t item);

  // Updates the in-memory/cached display attention policy immediately so the
  // display task reflects changes without requiring a reboot.
  /**
   * @brief Execute RuntimeSetDisplayAttentionPolicy.
   * @param policy Parameter policy.
   */
  void RuntimeSetDisplayAttentionPolicy(uint32_t policy);

  /**
   * @brief Execute RuntimeShowDisplayTestPattern.
   * @param duration_ms Parameter duration_ms.
   * @return Return the function result.
   */
  esp_err_t RuntimeShowDisplayTestPattern(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_MANAGER_H_
