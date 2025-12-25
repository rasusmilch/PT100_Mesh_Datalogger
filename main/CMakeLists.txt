idf_component_register(
  SRCS
    "app_main.c"
    "app_settings.c"
    "boot_mode.c"
    "calibration.c"
    "console_commands.c"
    "diagnostics/diag_common.c"
    "diagnostics/diag_fram.c"
    "diagnostics/diag_mesh.c"
    "diagnostics/diag_rtc.c"
    "diagnostics/diag_rtd.c"
    "diagnostics/diag_sd.c"
    "diagnostics/diag_wifi.c"
    "data_csv.c"
    "data_port.c"
    "net_stack.c"
    "crc16.c"
    "fram_i2c.c"
    "fram_log.c"
    "fram_spi.c"
    "i2c_bus.c"
    "max31865_reader.c"
    "pt100_table.c"
    "mesh_transport.c"
    "runtime_manager.c"
    "sd_csv_verify.c"
    "sd_logger.c"
    "wifi_service.c"
    "wifi_manager.c"
    "wifi_scan_wrap.c"
    "time_sync.c"
  INCLUDE_DIRS "."
  REQUIRES
    nvs_flash
    esp_netif
    esp_wifi
    esp_event
    esp_timer
    console
    vfs
    fatfs
    sdmmc
    driver
    esp_rom
    mbedtls
)

# Mesh-Lite invokes esp_wifi_scan_start() internally. We observed scan configs reaching the
# Wi-Fi driver with a non-zero/garbage channel bitmap, which triggers warnings like:
#   - "2g bitmap contains only invalid channels=..."
#   - "In 2G band, 5G channel bitmap does not take effect"
# Without modifying the mesh_lite component, wrap esp_wifi_scan_start() at link-time and
# sanitize the scan config to "scan all allowed channels" (bitmap=0) and always clear
# the 5G bitmap (ESP32-S3 is 2.4 GHz-only).

idf_build_get_property(project_elf EXECUTABLE)
if(project_elf)
  target_link_options(${project_elf} PRIVATE "-Wl,--wrap=esp_wifi_scan_start")
else()
  idf_build_set_property(LINK_OPTIONS "-Wl,--wrap=esp_wifi_scan_start" APPEND)
endif()
