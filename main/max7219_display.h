#ifndef PT100_LOGGER_MAX7219_DISPLAY_H_
#define PT100_LOGGER_MAX7219_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
    SemaphoreHandle_t spi_bus_mutex;
    TickType_t spi_bus_mutex_timeout_ticks;
  } max7219_display_config_t;

  typedef struct
  {
    spi_device_handle_t device;
    spi_host_device_t host;
    int chain_len;
    uint8_t intensity;
    bool initialized;
    SemaphoreHandle_t spi_bus_mutex;
    TickType_t spi_bus_mutex_timeout_ticks;
    uint32_t framebuffer[8];
  } max7219_display_t;

/**
 * @brief Execute Max7219DisplayInit.
 * @param disp Parameter disp.
 * @param config Parameter config.
 * @return Return the function result.
 */
  esp_err_t Max7219DisplayInit(max7219_display_t* disp,
                               const max7219_display_config_t* config);
/**
 * @brief Execute Max7219DisplayDeinit.
 * @param disp Parameter disp.
 * @return Return the function result.
 */
  esp_err_t Max7219DisplayDeinit(max7219_display_t* disp);

/**
 * @brief Execute Max7219DisplaySetText.
 * @param disp Parameter disp.
 * @param text Parameter text.
 */
  void Max7219DisplaySetText(max7219_display_t* disp, const char* text);

/**
 * @brief Execute Max7219DisplayClear.
 * @param disp Parameter disp.
 */
  void Max7219DisplayClear(max7219_display_t* disp);

/**
 * @brief Execute Max7219DisplaySetIntensity.
 * @param disp Parameter disp.
 * @param level_0_to_15 Parameter level_0_to_15.
 */
  void Max7219DisplaySetIntensity(max7219_display_t* disp,
                                  uint8_t level_0_to_15);

/**
 * @brief Execute Max7219DisplayShowTestPattern.
 * @param disp Parameter disp.
 */
  void Max7219DisplayShowTestPattern(max7219_display_t* disp);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_MAX7219_DISPLAY_H_
