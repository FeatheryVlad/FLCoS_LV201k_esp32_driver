#ifndef _PROCESSOR_DEFS_H
#define _PROCESSOR_DEFS_H

#define PIN_NUM_MISO 13 // 15 // 13
#define PIN_NUM_MOSI 11 // 16 // 11
#define PIN_NUM_SCLK  12 // 14 // 12
#define PIN_NUM_CS   10
#define PIN_NUM_RST  9

#define PIN_NUM_PCLK  8
#define PIN_NUM_HSYNC   16
#define PIN_NUM_VSYNC   15
#define PIN_NUM_VALID   14
#define PIN_NUM_DATA0   1
#define PIN_NUM_DATA1   2
#define PIN_NUM_DATA2   3
#define PIN_NUM_DATA3   4
#define PIN_NUM_DATA4   5
#define PIN_NUM_DATA5   7
#define PIN_NUM_DATA6   17
#define PIN_NUM_DATA7   6
#define LCD_H_RES       300
#define LCD_V_RES       224
#define LCD_V_RES       224

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (8) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_1_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (27000000) // Frequency in Hertz. Set frequency at 4 kHz


#endif /* _PROCESSOR_DEFS_H */