idf_component_register(
  SRCS
    "app_main.c"
    "app_settings.c"
    "calibration.c"
   
   
    "fram_log.c"
    "fram_spi.c"
    "max31865_reader.c"
    "mesh_transport.c"
    "sd_logger.c"
    "time_sync.c"
  INCLUDE_DIRS "."
  REQUIRES
    nvs_flash
    esp_netif
    esp_wifi
    esp_event
    esp_timer
    esp_console
    vfs
    fatfs
    sdmmc
    driver
    esp_rom
)


)
