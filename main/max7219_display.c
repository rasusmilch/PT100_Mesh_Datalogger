#include "max7219_display.h"

#include <ctype.h>
#include <string.h>

#include "esp_log.h"

static const char* kTag = "max7219";

typedef struct
{
  char c;
  uint8_t rows[7];
} font_glyph_t;

static const font_glyph_t kFont5x7[] = {
  { '0', { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E } },
  // 00100
  // 01100
  // 00100
  // 00100
  // 00100
  // 00100
  // 01110
  { '1', { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E } },
  { '2', { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F } },
  { '3', { 0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E } },
  { '4', { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 } },
  { '5', { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E } },
  { '6', { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E } },
  { '7', { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 } },
  { '8', { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E } },
  { '9', { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C } },
  { 'A', { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
  { 'B', { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E } },
  { 'C', { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E } },
  { 'D', { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E } },
  { 'E', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F } },
  { 'F', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 } },
  { 'G', { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E } },
  { 'H', { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
  { 'I', { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E } },
  { 'J', { 0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C } },
  { 'K', { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 } },
  { 'L', { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F } },
  { 'M', { 0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11 } },
  { 'N', { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 } },
  { 'O', { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
  { 'P', { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 } },
  { 'Q', { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D } },
  { 'R', { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 } },
  { 'S', { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E } },
  { 'T', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 } },
  { 'U', { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
  { 'V', { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 } },
  { 'W', { 0x11, 0x11, 0x11, 0x11, 0x15, 0x1B, 0x11 } },
  { 'X', { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 } },
  { 'Y', { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 } },
  { 'Z', { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F } },
  { '-', { 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00 } },
  { '.', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06 } },
  { ' ', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
};

/**
 * @brief Execute FindGlyph.
 * @param c Parameter c.
 * @return Return the function result.
 */
static const font_glyph_t*
FindGlyph(char c)
{
  for (size_t i = 0; i < sizeof(kFont5x7) / sizeof(kFont5x7[0]); ++i) {
    if (kFont5x7[i].c == c) {
      return &kFont5x7[i];
    }
  }
  return NULL;
}

/**
 * @brief Execute TransformCoords.
 * @param x Parameter x.
 * @param y Parameter y.
 */
static void
TransformCoords(int* x, int* y)
{
  int tx = *x;
  int ty = *y;

#if CONFIG_APP_MAX7219_ROTATE_180
  tx = 31 - tx;
  ty = 7 - ty;
#endif

#if CONFIG_APP_MAX7219_MIRROR_X
  tx = 31 - tx;
#endif

#if CONFIG_APP_MAX7219_MIRROR_Y
  ty = 7 - ty;
#endif

  *x = tx;
  *y = ty;
}

/**
 * @brief Execute SetPixel.
 * @param disp Parameter disp.
 * @param x Parameter x.
 * @param y Parameter y.
 * @param on Parameter on.
 */
static void
SetPixel(max7219_display_t* disp, int x, int y, bool on)
{
  if (disp == NULL) {
    return;
  }
  if (x < 0 || x >= 32 || y < 0 || y >= 8) {
    return;
  }

  TransformCoords(&x, &y);

  if (x < 0 || x >= 32 || y < 0 || y >= 8) {
    return;
  }

  if (on) {
    disp->framebuffer[y] |= (1u << (uint32_t)x);
  } else {
    disp->framebuffer[y] &= ~(1u << (uint32_t)x);
  }
}

static void
DrawCompressedDecimal(max7219_display_t* disp, int start_x)
{
  const int dot_left_x = start_x + 1;
  const int dot_right_x = start_x + 2;

  if (disp == NULL) {
    return;
  }
  if (start_x < 0 || (start_x + 3) >= 32) {
    return;
  }

  SetPixel(disp, dot_left_x, 5, true);
  SetPixel(disp, dot_left_x, 6, true);
  SetPixel(disp, dot_right_x, 5, true);
  SetPixel(disp, dot_right_x, 6, true);
}

/**
 * @brief Execute TransmitWithLock.
 * @param disp Parameter disp.
 * @param transaction Parameter transaction.
 * @return Return the function result.
 */
static esp_err_t
TransmitWithLock(max7219_display_t* disp, spi_transaction_t* transaction)
{
  if (disp == NULL || disp->device == NULL || transaction == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (disp->spi_bus_mutex != NULL) {
    if (xSemaphoreTake(disp->spi_bus_mutex,
                       disp->spi_bus_mutex_timeout_ticks) != pdTRUE) {
      return ESP_ERR_TIMEOUT;
    }
  }

  esp_err_t result = spi_device_transmit(disp->device, transaction);

  if (disp->spi_bus_mutex != NULL) {
    (void)xSemaphoreGive(disp->spi_bus_mutex);
  }
  return result;
}

/**
 * @brief Execute WriteRegisterAll.
 * @param disp Parameter disp.
 * @param reg Parameter reg.
 * @param value Parameter value.
 * @return Return the function result.
 */
static esp_err_t
WriteRegisterAll(max7219_display_t* disp, uint8_t reg, uint8_t value)
{
  if (disp == NULL || !disp->initialized || disp->device == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  const int chain_len = disp->chain_len;
  if (chain_len <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t tx_buf[32] = { 0 };
  const int tx_len = chain_len * 2;
  if (tx_len > (int)sizeof(tx_buf)) {
    return ESP_ERR_INVALID_SIZE;
  }

  for (int dev = 0; dev < chain_len; ++dev) {
    tx_buf[dev * 2] = reg;
    tx_buf[dev * 2 + 1] = value;
  }

  spi_transaction_t t = {
    .length = tx_len * 8,
    .tx_buffer = tx_buf,
  };

  return TransmitWithLock(disp, &t);
}

/**
 * @brief Execute FlushFramebuffer.
 * @param disp Parameter disp.
 * @return Return the function result.
 */
static esp_err_t
FlushFramebuffer(max7219_display_t* disp)
{
  if (disp == NULL || !disp->initialized || disp->device == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  const int chain_len = disp->chain_len;
  if (chain_len <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t tx_buf[32] = { 0 };
  const int tx_len = chain_len * 2;
  if (tx_len > (int)sizeof(tx_buf)) {
    return ESP_ERR_INVALID_SIZE;
  }

  for (int row = 0; row < 8; ++row) {
    int tx_index = 0;
    for (int dev = chain_len - 1; dev >= 0; --dev) {
      uint8_t value = 0;
#if CONFIG_APP_MAX7219_REVERSE_MODULE_ORDER
      const int x_base = (chain_len - 1 - dev) * 8;
#else
      const int x_base = dev * 8;
#endif
      for (int bit = 0; bit < 8; ++bit) {
        const int x = x_base + bit;
        const bool on = (disp->framebuffer[row] & (1u << (uint32_t)x)) != 0;
        if (on) {
          value |= (uint8_t)(1u << (7 - bit));
        }
      }
      tx_buf[tx_index++] = (uint8_t)(row + 1);
      tx_buf[tx_index++] = value;
    }

    spi_transaction_t t = {
      .length = tx_len * 8,
      .tx_buffer = tx_buf,
    };
    esp_err_t result = TransmitWithLock(disp, &t);
    if (result != ESP_OK) {
      return result;
    }
  }

  return ESP_OK;
}

/**
 * @brief Execute Max7219DisplayInit.
 * @param disp Parameter disp.
 * @param config Parameter config.
 * @return Return the function result.
 */
esp_err_t
Max7219DisplayInit(max7219_display_t* disp,
                   const max7219_display_config_t* config)
{
  if (disp == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(disp, 0, sizeof(*disp));
  disp->chain_len = config->chain_len;
  disp->host = config->host;
  disp->intensity = config->intensity;
  disp->spi_bus_mutex = config->spi_bus_mutex;
  disp->spi_bus_mutex_timeout_ticks = config->spi_bus_mutex_timeout_ticks;

#if !CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
  bool bus_initialized = false;
  spi_bus_config_t bus_config = {
    .mosi_io_num = config->mosi_gpio,
    .miso_io_num = -1,
    .sclk_io_num = config->sclk_gpio,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 256,
  };

  esp_err_t bus_result =
    spi_bus_initialize(config->host, &bus_config, SPI_DMA_CH_AUTO);
  if (bus_result != ESP_OK) {
    ESP_LOGE(
      kTag, "spi_bus_initialize failed: %s", esp_err_to_name(bus_result));
    return bus_result;
  }
  bus_initialized = true;
#endif

  spi_device_interface_config_t dev_config = {
    .clock_speed_hz = (int)config->clock_hz,
    .mode = 0,
    .spics_io_num = config->cs_gpio,
    .queue_size = 1,
  };

  esp_err_t dev_result =
    spi_bus_add_device(config->host, &dev_config, &disp->device);
  if (dev_result != ESP_OK) {
    if (dev_result == ESP_ERR_INVALID_STATE) {
      ESP_LOGE(kTag,
               "SPI bus not initialized before MAX7219 init; expected "
               "RuntimeManagerInit to call InitSpiBus() first.");
    } else {
      ESP_LOGE(
        kTag, "spi_bus_add_device failed: %s", esp_err_to_name(dev_result));
    }
#if !CONFIG_APP_MAX7219_SHARE_APP_SPI_BUS
    if (bus_initialized) {
      (void)spi_bus_free(config->host);
    }
#endif
    return dev_result;
  }

  disp->initialized = true;

  (void)WriteRegisterAll(disp, 0x0F, 0x00); // display test off
  (void)WriteRegisterAll(disp, 0x09, 0x00); // no decode
  (void)WriteRegisterAll(disp, 0x0B, 0x07); // scan limit 7
  (void)WriteRegisterAll(disp, 0x0A, disp->intensity & 0x0F);
  (void)WriteRegisterAll(disp, 0x0C, 0x01); // shutdown=0

  memset(disp->framebuffer, 0, sizeof(disp->framebuffer));
  return FlushFramebuffer(disp);
}

/**
 * @brief Execute Max7219DisplayDeinit.
 * @param disp Parameter disp.
 * @return Return the function result.
 */
esp_err_t
Max7219DisplayDeinit(max7219_display_t* disp)
{
  if (disp == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t result = ESP_OK;
  if (disp->device != NULL) {
    result = spi_bus_remove_device(disp->device);
  }
  memset(disp, 0, sizeof(*disp));
  return result;
}

/**
 * @brief Execute Max7219DisplaySetText.
 * @param disp Parameter disp.
 * @param text Parameter text.
 */
void
Max7219DisplaySetText(max7219_display_t* disp, const char* text)
{
  if (disp == NULL || !disp->initialized) {
    return;
  }

  memset(disp->framebuffer, 0, sizeof(disp->framebuffer));

  if (text == NULL) {
    (void)FlushFramebuffer(disp);
    return;
  }

  int cursor_x = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    const char current_char = *p;
    const char next_char = *(p + 1);

    if (current_char == '.') {
      if ((cursor_x + 3) >= 32) {
        break;
      }
      DrawCompressedDecimal(disp, cursor_x);
      cursor_x += 4;
      if (cursor_x >= 32) {
        break;
      }
      continue;
    }

    const char glyph_char = (char)toupper((unsigned char)current_char);
    const font_glyph_t* glyph = FindGlyph(glyph_char);
    if (glyph == NULL) {
      glyph = FindGlyph(' ');
    }
    if (glyph == NULL) {
      continue;
    }

    for (int row = 0; row < 7; ++row) {
      const uint8_t row_bits = glyph->rows[row];
      for (int col = 0; col < 5; ++col) {
        const bool on = ((row_bits >> (4 - col)) & 0x01) != 0;
        SetPixel(disp, cursor_x + col, row, on);
      }
    }

    int advance_cols = 6;
    if (next_char == '.') {
      advance_cols = 5;
    }

    cursor_x += advance_cols;
    if (cursor_x >= 32) {
      break;
    }
  }

  (void)FlushFramebuffer(disp);
}

/**
 * @brief Execute Max7219DisplayShowTestPattern.
 * @param disp Parameter disp.
 */
void
Max7219DisplayShowTestPattern(max7219_display_t* disp)
{
  Max7219DisplaySetText(disp, "ABCDE");
}

/**
 * @brief Execute Max7219DisplayClear.
 * @param disp Parameter disp.
 */
void
Max7219DisplayClear(max7219_display_t* disp)
{
  if (disp == NULL || !disp->initialized) {
    return;
  }
  memset(disp->framebuffer, 0, sizeof(disp->framebuffer));
  (void)FlushFramebuffer(disp);
}

/**
 * @brief Execute Max7219DisplaySetIntensity.
 * @param disp Parameter disp.
 * @param level_0_to_15 Parameter level_0_to_15.
 */
void
Max7219DisplaySetIntensity(max7219_display_t* disp, uint8_t level_0_to_15)
{
  if (disp == NULL || !disp->initialized) {
    return;
  }
  disp->intensity = level_0_to_15 & 0x0F;
  (void)WriteRegisterAll(disp, 0x0A, disp->intensity);
}
