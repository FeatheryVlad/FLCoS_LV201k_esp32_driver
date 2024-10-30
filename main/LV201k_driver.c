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
  //while (1){
  //}
  vTaskDelete(NULL);
}

//static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
//{
//    BaseType_t high_task_awoken = pdFALSE;
////#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
////    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
////        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
////    }
////#endif
//    return high_task_awoken == pdTRUE;
//}

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

static SemaphoreHandle_t lvgl_mux = NULL;

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

int color = 0x000000;


static lv_color_t *my_color_map1 = NULL;

int x=0;
int y=0;
int box_size = 10;
int x_velocity=1;
int y_velocity=1;

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
    .gpio_num = PIN_NUM_PCLK,
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
  //xTaskCreate(lvd_spi_task, "lvd_spi_task", 2048*3, NULL, 1, NULL);
  for (size_t j = 0; j < 2; j++){
    ESP_LOGI(TAG, "Recv str");
    for (uint8_t i = 0; i < 16; i++){
      ESP_ERROR_CHECK(lvd_SPI_read(i, PIN_NUM_CS));
      //vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  //vTaskDelay(500 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK(ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0));
  ESP_LOGI(TAG, "Install RGB LCD panel driver");
  //#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  //  ESP_LOGI(TAG, "Create semaphores");
  //  sem_vsync_end = xSemaphoreCreateBinary();
  //  assert(sem_vsync_end);
  //  sem_gui_ready = xSemaphoreCreateBinary();
  //  assert(sem_gui_ready);
  //#endif
  static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;      // contains callback functions
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
      .data_width = 8, // RGB565 in parallel mode, thus 16bit in width
      //.psram_trans_align = 64,
      //.num_fbs = EXAMPLE_LCD_NUM_FB,
      .clk_src = LCD_CLK_SRC_DEFAULT,
      .disp_gpio_num = PIN_NUM_VALID,
      .pclk_gpio_num = PIN_NUM_PCLK,
      .vsync_gpio_num = PIN_NUM_VSYNC,
      .hsync_gpio_num = PIN_NUM_HSYNC,
      .de_gpio_num = -1,
      .data_gpio_nums = {
          PIN_NUM_DATA0,
          PIN_NUM_DATA1,
          PIN_NUM_DATA2,
          PIN_NUM_DATA3,
          PIN_NUM_DATA4,
          PIN_NUM_DATA5,
          PIN_NUM_DATA6,
          PIN_NUM_DATA7,
      },
      .timings = {
          .pclk_hz = LEDC_FREQUENCY,
          .h_res = LCD_H_RES,
          .v_res = LCD_V_RES,
          // The following paramete rs should refer to LCD spec
          .vsync_front_porch = 3,
          .vsync_pulse_width = 3,
          .vsync_back_porch = 16,
          .hsync_front_porch = 572,
          .hsync_pulse_width = 126,
          .hsync_back_porch = 118,
          //.flags.pclk_active_neg = true,
      },
      .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
  //ESP_LOGI(TAG, "Register event callbacks");
  //esp_lcd_rgb_panel_event_callbacks_t cbs = {
  //    .on_vsync = example_on_vsync_event,
  //};
  //ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

  //ESP_LOGI(TAG, "Initialize RGB LCD panel");
  //ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  //ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_LOGI(TAG, "Register event callbacks");
  //esp_lcd_rgb_panel_event_callbacks_t cbs = {
  //    .on_vsync = example_on_vsync_event,
  //};
  //ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

  ESP_LOGI(TAG, "Initialize RGB LCD panel");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

//#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
//    ESP_LOGI(TAG, "Turn on LCD backlight");
//    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
//#endif

  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();

    my_color_map1 = (lv_color_t *) heap_caps_malloc(box_size * box_size  * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(my_color_map1);

    for (int i=0; i<(box_size+1) * (box_size+1); i++) {
        my_color_map1[i] = lv_color_hsv_to_rgb(color, 80, 80);
    }


    x=box_size/2+5;
    y=box_size/2+5;

    int size = (box_size+1) * (box_size+1) * sizeof(lv_color_t);


    while (1) {

        for (int i=0; i<50; i++) {
            esp_lcd_panel_draw_bitmap(panel_handle, x-box_size/2, y-box_size/2, x-box_size/2+box_size+1, y-box_size/2+box_size+1, my_color_map1);
            // ESP_LOGI(TAG, "finished copy");
            if ( (x < box_size/2+5) || (x > LCD_H_RES- box_size/2-5) ) {
                x_velocity *= -1;
            }
            if ( (y < box_size/2+5) || (y > LCD_V_RES- box_size/2-5) ) {
                y_velocity *= -1;
            }

            x += x_velocity;
            y += y_velocity;

            vTaskDelay(pdMS_TO_TICKS(2)); 
             ESP_LOGI(TAG, "x: %d, y: %d, color: %d, i: %d", x, y, color,i );
        }

        if (color > 330) {
            color = 0;
        }
        else {
            color +=20;
        }

        for (int i=0; i<(box_size+1)*(box_size+1); i++) {
            my_color_map1[i] = lv_color_hsv_to_rgb(color, 80, 80);
        }


    }
  //void *buf1 = NULL;
  //void *buf2 = NULL;
  
  //ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
  //buf1 = heap_caps_malloc(LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  //assert(buf1);
  //// initialize LVGL draw buffers
  //lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 100);

  //ESP_LOGI(TAG, "Register display driver to LVGL");
  //lv_disp_drv_init(&disp_drv);
  //disp_drv.hor_res = LCD_H_RES;
  //disp_drv.ver_res = LCD_V_RES;
  //disp_drv.flush_cb = example_lvgl_flush_cb;
  //disp_drv.draw_buf = &disp_buf;
  //disp_drv.user_data = panel_handle;

  //lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  //ESP_LOGI(TAG, "Install LVGL tick timer");
  //// Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
  //const esp_timer_create_args_t lvgl_tick_timer_args = {
  //    .callback = &example_increase_lvgl_tick,
  //    .name = "lvgl_tick"
  //};
  //esp_timer_handle_t lvgl_tick_timer = NULL;
  //ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  //ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

  //lvgl_mux = xSemaphoreCreateRecursiveMutex();
  //assert(lvgl_mux);
  //ESP_LOGI(TAG, "Create LVGL task");
  //xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

  //ESP_LOGI(TAG, "Display LVGL Scatter Chart");
  //// Lock the mutex due to the LVGL APIs are not thread-safe
  //if (example_lvgl_lock(-1)) {
  //  example_lvgl_demo_ui(disp);
  //  // Release the mutex
  //  example_lvgl_unlock();
  //}
  /**
   * 
E (50574) task_wdt:  - IDLE0 (CPU 0)
E (50574) task_wdt: Tasks currently running:
E (50574) task_wdt: CPU 0: main
E (50574) task_wdt: CPU 1: IDLE1
E (50574) task_wdt: Print CPU 0 (current core) backtrace


Backtrace: 0x42016CB3:0x3FC960D0 0x420170D0:0x3FC960F0 0x4037779D:0x3FC96120 0x40048838:0x3FC9C2D0 0x40048C69:0x3FC9C2F0 0x40043D03:0x3FC9C310 0x40043CD5:0x3FC9C330 0x40044183:0x3FC9C350 0x40044281:0x3FC9C3D0 0x4037A956:0x3FC9C420 0x4200A157:0x3FC9C460 0x4200959E:0x3FC9C4B0 0x4200942B:0x3FC9C4E0 0x42008F91:0x3FC9C5F0 0x420266D3:0x3FC9C610 0x4037C241:0x3FC9C640
0x42016cb3: task_wdt_timeout_handling at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/esp_system/task_wdt/task_wdt.c:441
0x420170d0: task_wdt_isr at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/esp_system/task_wdt/task_wdt.c:515
0x4037779d: _xt_lowint1 at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/xtensa/xtensa_vectors.S:1240
0x40048838: uart_tx_one_char_uart in ROM
0x40048c69: uart_tx_one_char in ROM
0x40043d03: ets_write_char_uart in ROM
0x40043cd5: ets_write_char in ROM
0x40044183: ets_vprintf in ROM
0x40044281: ets_printf in ROM
0x4037a956: esp_cache_msync at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/esp_mm/esp_cache.c:56 (discriminator 1)
0x4200a157: rgb_panel_draw_bitmap at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/esp_lcd/src/esp_lcd_panel_rgb.c:752
0x4200959e: esp_lcd_panel_draw_bitmap at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/esp_lcd/src/esp_lcd_panel_ops.c:34
0x4200942b: lvd_initialize at E:/ESP_Projects/AlphaLens/main/LV201k_driver.c:351
0x42008f91: app_main at E:/ESP_Projects/AlphaLens/main/app_main.c:22
0x420266d3: main_task at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/freertos/app_startup.c:208
0x4037c241: vPortTaskWrapper at E:/Espressif_IDF/esp-idfs/esp-idf-v5.2.1/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:134


start address: 0x3c059370, or the size: 0xce4 is(are) not aligned with cache line size (0x20)B
   */
}