#include "LV201k_driver.h"
#include "proc_defs.h"

#define DEBUG

spi_device_handle_t spi;

// https://forum.arduino.cc/t/custom-spi-pins-with-esp32-are-not-connecting-to-sd-card/976940/10

static const char *TAG = "LV201K_SPI";

esp_err_t lvd_init_spi(int spiCLK, int spiMOSI, int spiMISO, int csPin){
  esp_err_t ret;
  spi_bus_config_t bus_config = {
    .sclk_io_num = spiCLK,
    .mosi_io_num = spiMOSI,
    .miso_io_num = spiMISO,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
  };
  spi_device_interface_config_t dev_config = {
    .address_bits = 0,
    .command_bits = 0,
    .dummy_bits = 0,
    .mode = 3,
    .duty_cycle_pos = 0,
    .cs_ena_posttrans = 0,
    .cs_ena_pretrans = 0,
    .clock_speed_hz = 320000,
    .spics_io_num = csPin,//CS pin not used
    .flags = 0,
    .queue_size = 1,
    .pre_cb = NULL,
    .post_cb = NULL
  };
  ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
  if(ret != ESP_OK) return ret;
  ret = spi_bus_add_device(SPI2_HOST, &dev_config, &spi);
  return ret;
}

uint8_t txData[2] = {};
uint8_t rxData[25] = {};

esp_err_t lvd_SPI_write(uint8_t address, uint8_t sendData, int csPin){
  esp_err_t ret;
  txData[0] = address & 0x7F;
  txData[1] = sendData;
  //gpio_set_level(csPin, 0);
  spi_transaction_t trans_desc = {
    .cmd = 0,
    .addr = 0,
    .length = (8 * 2),
    .tx_buffer = txData,
    .rxlength = (8 * 2),
    .rx_buffer = rxData,
    .flags = 0
  };
  //memset(&trans_desc, 0, sizeof(trans_desc));
  ret = spi_device_transmit(spi, &trans_desc);
  #ifdef DEBUG
    printf("%d\n", txData[1]);
    //printf("%d\n", rxData[0]);
    //printf("%d\n", rxData[1]);
    //printf("%d\n", rxData[2]);
  #endif
  //gpio_set_level(csPin, 1);
  return ret;
}

esp_err_t lvd_SPI_read(uint8_t address, int csPin){
  esp_err_t ret = 0;
  txData[0] = address | 0x80;
  txData[1] = 0x00;
  //gpio_set_level(csPin, 0);
  //memset(&trans_desc, 0, sizeof(trans_desc));
  spi_transaction_t trans_desc = {
    .cmd = 0,
    .addr = 0,
    .length = (8 * 3),
    .tx_buffer = txData,
    .rxlength = (8 * 2),
    .rx_buffer = rxData,
    .flags = 0
  };
  ret = spi_device_transmit(spi, &trans_desc);
  //gpio_set_level(csPin, 1);
  #ifdef DEBUG
    printf("reg%X: %0X\n", address, rxData[1]);
  #endif
  return ret; 
}

void lvd_spi_task(void *arg){
  while (1){
    ESP_LOGI(TAG, "Recv str");
    for (uint8_t i = 0; i < 16; i++){
      ESP_ERROR_CHECK(lvd_SPI_read(i, PIN_NUM_CS));
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void lvd_initialize(int spiCLK, int spiMOSI, int spiMISO){
  gpio_config_t io_conf = {
    .pin_bit_mask = ((1ULL << PIN_NUM_RST) | (1ULL << PIN_NUM_CS)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = true
  };
  gpio_config(&io_conf);
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = LEDC_DUTY_RES,
    .timer_num = LEDC_TIMER,
    .freq_hz = LEDC_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL,
    .timer_sel = LEDC_TIMER,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = LEDC_OUTPUT_IO,
    .duty = 2, // Set duty to 0%
    .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  //Reset the display
  gpio_set_level(PIN_NUM_RST, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gpio_set_level(PIN_NUM_RST, 1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  ESP_ERROR_CHECK(lvd_init_spi(spiCLK, spiMOSI, spiMISO, PIN_NUM_CS));

  uint8_t cmd = 0;
  gpio_set_level(PIN_NUM_CS, 1);
  ESP_LOGI(TAG, "start Recv");
  while (preset_lvd_regs[cmd].databytes != 0xff){
    (lvd_SPI_write(preset_lvd_regs[cmd].cmd, preset_lvd_regs[cmd].data, PIN_NUM_CS));
    cmd++;
  }
  ESP_LOGI(TAG, "pre end Recv");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "end Recv");
  xTaskCreate(lvd_spi_task, "lvd_spi_task", 2048*3, NULL, 1, NULL);
}