#ifndef PT100_LOGGER_MAX7219_DISPLAY_H_
#define PT100_LOGGER_MAX7219_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    spi_host_device_t host;
    int mosi_gpio;
    int sclk_gpio;
    int cs_gpio;
    int chain_len;
    uint32_t clock_hz;
    uint8_t intensity;
  } max7219_display_config_t;

  typedef struct
  {
    spi_device_handle_t device;
    spi_host_device_t host;
    int chain_len;
    uint8_t intensity;
    bool initialized;
    uint32_t framebuffer[8];
  } max7219_display_t;

  esp_err_t Max7219DisplayInit(max7219_display_t* disp,
                               const max7219_display_config_t* config);

  void Max7219DisplaySetText(max7219_display_t* disp, const char* text);

  void Max7219DisplayClear(max7219_display_t* disp);

  void Max7219DisplaySetIntensity(max7219_display_t* disp,
                                  uint8_t level_0_to_15);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MAX7219_DISPLAY_H_
