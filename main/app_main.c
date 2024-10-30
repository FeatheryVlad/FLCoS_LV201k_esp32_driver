
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "esp_log.h"
#include "proc_defs.h"
#include "LV201k_driver.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "_MAIN";

void app_main(void)
{
  esp_err_t ret;
  lvd_initialize(PIN_NUM_SCLK, PIN_NUM_MOSI, PIN_NUM_MISO);
}