// Display Library for SPI e-paper panels from Dalian Good Display and boards from Waveshare.
// Requires ESP-IDF SPI and GPIO. Caution: the e-paper panels require 3.3V supply AND data lines!
//
// Display Library based on Demo Example from Good Display: https://www.good-display.com/companyfile/32/
//
// Author: Jean-Marc Zingg (ported to ESP-IDF)
//
// Version: see library.properties
//
// Library: https://github.com/ZinggJM/GxEPD2

#include "GxEPD2_EPD.h"

const char* GxEPD2_EPD::TAG = "GxEPD2";

GxEPD2_EPD::GxEPD2_EPD(int16_t cs, int16_t dc, int16_t rst, int16_t busy, int16_t busy_level, uint32_t busy_timeout,
                       uint16_t w, uint16_t h, GxEPD2::Panel p, bool c, bool pu, bool fpu) :
  WIDTH(w), HEIGHT(h), panel(p), hasColor(c), hasPartialUpdate(pu), hasFastPartialUpdate(fpu),
  _cs(cs), _dc(dc), _rst(rst), _busy(busy), _busy_level(busy_level), _busy_timeout(busy_timeout), _diag_enabled(false),
  _spi_host(HSPI_HOST), _devcfg({.command_bits = 0, .address_bits = 0, .dummy_bits = 0, .mode = 0, .duty_cycle_pos = 0, .cs_ena_pretrans = 0, .cs_ena_posttrans = 0, .clock_speed_hz = 10000000, .input_delay_ns = 0, .spics_io_num = cs, .flags = 0, .queue_size = 7, .pre_cb = NULL, .post_cb = NULL}), _spi_handle(NULL)
{
  _initial_write = true;
  _initial_refresh = true;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _init_display_done = false;
  _reset_duration = 10;
  _busy_callback = 0;
  _busy_callback_parameter = 0;
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate)
{
  init(serial_diag_bitrate, true, 10, false);
}

void GxEPD2_EPD::init(uint32_t serial_diag_bitrate, bool initial, uint16_t reset_duration, bool pulldown_rst_mode)
{
  _initial_write = initial;
  _initial_refresh = initial;
  _pulldown_rst_mode = pulldown_rst_mode;
  _power_is_on = false;
  _using_partial_mode = false;
  _hibernating = false;
  _init_display_done = false;
  _reset_duration = reset_duration;
  if (serial_diag_bitrate > 0)
  {
    // ESP-IDF logging is always enabled; bitrate ignored
    _diag_enabled = true;
  }
  // Initialize GPIO pins
  if (_cs >= 0)
  {
    gpio_set_direction((gpio_num_t)_cs, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_cs, 1);
  }
  _reset();
  // Initialize SPI bus
  spi_bus_config_t buscfg = {
    .miso_io_num = 12, // Default ESP32 HSPI MISO
    .mosi_io_num = 13, // Default ESP32 HSPI MOSI
    .sclk_io_num = 14, // Default ESP32 HSPI SCLK
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4094
  };
  esp_err_t ret = spi_bus_initialize(_spi_host, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  ret = spi_bus_add_device(_spi_host, &_devcfg, &_spi_handle);
  ESP_ERROR_CHECK(ret);
  if (_rst >= 0)
  {
    gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_rst, 1);
  }
  if (_dc >= 0)
  {
    gpio_set_direction((gpio_num_t)_dc, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)_dc, 1);
  }
  if (_busy >= 0)
  {
    gpio_set_direction((gpio_num_t)_busy, GPIO_MODE_INPUT);
  }
}

void GxEPD2_EPD::end()
{
  spi_bus_remove_device(_spi_handle);
  spi_bus_free(_spi_host);
  if (_cs >= 0) gpio_set_direction((gpio_num_t)_cs, GPIO_MODE_INPUT);
  if (_dc >= 0) gpio_set_direction((gpio_num_t)_dc, GPIO_MODE_INPUT);
  if (_rst >= 0) gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_INPUT);
}

void GxEPD2_EPD::setBusyCallback(void (*busyCallback)(const void*), const void* busy_callback_parameter)
{
  _busy_callback = busyCallback;
  _busy_callback_parameter = busy_callback_parameter;
}

void GxEPD2_EPD::selectSPI(spi_host_device_t host, spi_device_interface_config_t devcfg)
{
  _spi_host = host;
  _devcfg = devcfg;
}

void GxEPD2_EPD::_reset()
{
  if (_rst >= 0)
  {
    if (_pulldown_rst_mode)
    {
      gpio_set_level((gpio_num_t)_rst, 0);
      gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_OUTPUT);
      gpio_set_level((gpio_num_t)_rst, 0);
      vTaskDelay(pdMS_TO_TICKS(_reset_duration));
      gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_INPUT_PULLUP);
      vTaskDelay(pdMS_TO_TICKS(_reset_duration > 10 ? _reset_duration : 10));
    }
    else
    {
      gpio_set_level((gpio_num_t)_rst, 1);
      gpio_set_direction((gpio_num_t)_rst, GPIO_MODE_OUTPUT);
      gpio_set_level((gpio_num_t)_rst, 1);
      vTaskDelay(pdMS_TO_TICKS(10));
      gpio_set_level((gpio_num_t)_rst, 0);
      vTaskDelay(pdMS_TO_TICKS(_reset_duration));
      gpio_set_level((gpio_num_t)_rst, 1);
      vTaskDelay(pdMS_TO_TICKS(_reset_duration > 10 ? _reset_duration : 10));
    }
    _hibernating = false;
  }
}

void GxEPD2_EPD::_waitWhileBusy(const char* comment, uint16_t busy_time)
{
  if (_busy >= 0)
  {
    vTaskDelay(pdMS_TO_TICKS(1)); // add some margin
    unsigned long start = esp_timer_get_time() / 1000; // in ms
    while (1)
    {
      if (gpio_get_level((gpio_num_t)_busy) != _busy_level) break;
      if (_busy_callback) _busy_callback(_busy_callback_parameter);
      else vTaskDelay(pdMS_TO_TICKS(1));
      if (gpio_get_level((gpio_num_t)_busy) != _busy_level) break;
      if ((esp_timer_get_time() / 1000 - start) > _busy_timeout)
      {
        ESP_LOGE(TAG, "Busy Timeout!");
        break;
      }
#if defined(ESP_PLATFORM)
      vTaskDelay(pdMS_TO_TICKS(1)); // yield to avoid WDT
#endif
    }
    if (comment)
    {
#if !defined(DISABLE_DIAGNOSTIC_OUTPUT)
      if (_diag_enabled)
      {
        unsigned long elapsed = esp_timer_get_time() / 1000 - start;
        ESP_LOGI(TAG, "%s : %lu", comment, elapsed);
      }
#endif
    }
    (void) start;
  }
  else vTaskDelay(pdMS_TO_TICKS(busy_time));
}

void GxEPD2_EPD::_writeCommand(uint8_t c)
{
  spi_transaction_t t = {};
  t.length = 8;
  t.tx_buffer = &c;
  if (_dc >= 0) gpio_set_level((gpio_num_t)_dc, 0);
  esp_err_t ret = spi_device_transmit(_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
  if (_dc >= 0) gpio_set_level((gpio_num_t)_dc, 1);
}

void GxEPD2_EPD::_writeData(uint8_t d)
{
  spi_transaction_t t = {};
  t.length = 8;
  t.tx_buffer = &d;
  esp_err_t ret = spi_device_transmit(_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
}

void GxEPD2_EPD::_writeData(const uint8_t* data, uint16_t n)
{
  spi_transaction_t t = {};
  t.length = n * 8;
  t.tx_buffer = data;
  esp_err_t ret = spi_device_transmit(_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
}

void GxEPD2_EPD::_writeDataPGM(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  // In ESP-IDF, const data is accessible directly (no PROGMEM)
  uint8_t* buffer = (uint8_t*)malloc(n + fill_with_zeroes);
  if (!buffer) return;
  for (uint16_t i = 0; i < n; i++)
  {
    buffer[i] = *data++;  // Direct access
  }
  for (int16_t i = 0; i < fill_with_zeroes; i++)
  {
    buffer[n + i] = 0x00;
  }
  _writeData(buffer, n + fill_with_zeroes);
  free(buffer);
}

void GxEPD2_EPD::_writeDataPGM_sCS(const uint8_t* data, uint16_t n, int16_t fill_with_zeroes)
{
  // Simplified: handle CS manually if needed, but ESP-IDF manages it
  _writeDataPGM(data, n, fill_with_zeroes);
}

void GxEPD2_EPD::_writeCommandData(const uint8_t* pCommandData, uint8_t datalen)
{
  spi_transaction_t t = {};
  t.length = (datalen) * 8;
  t.tx_buffer = pCommandData + 1; // Skip command byte
  if (_dc >= 0) gpio_set_level((gpio_num_t)_dc, 0);
  esp_err_t ret = spi_device_transmit(_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
  if (_dc >= 0) gpio_set_level((gpio_num_t)_dc, 1);
  // Send data part
  t.length = (datalen - 1) * 8;
  t.tx_buffer = pCommandData + 1;
  ret = spi_device_transmit(_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
}

void GxEPD2_EPD::_writeCommandDataPGM(const uint8_t* pCommandData, uint8_t datalen)
{
  uint8_t* buffer = (uint8_t*)malloc(datalen);
  if (!buffer) return;
  for (uint8_t i = 0; i < datalen; i++)
  {
    buffer[i] = *pCommandData++;  // Direct access
  }
  _writeCommandData(buffer, datalen);
  free(buffer);
}

void GxEPD2_EPD::_startTransfer()
{
  // ESP-IDF SPI handles transfers; no explicit start needed
}

void GxEPD2_EPD::_transfer(uint8_t value)
{
  spi_transaction_t t = {};
  t.length = 8;
  t.tx_buffer = &value;
  esp_err_t ret = spi_device_transmit(_spi_handle, &t);
  ESP_ERROR_CHECK(ret);
}

void GxEPD2_EPD::_endTransfer()
{
  // ESP-IDF SPI handles CS automatically
}
