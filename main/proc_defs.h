#ifndef _PROCESSOR_DEFS_H
#define _PROCESSOR_DEFS_H

#define PIN_NUM_MISO 13 // 15 // 13
#define PIN_NUM_MOSI 11 // 16 // 11
#define PIN_NUM_CLK  12 // 14 // 12
#define PIN_NUM_CS   10
#define PIN_NUM_RST  9

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (8) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_1_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (25000000) // Frequency in Hertz. Set frequency at 4 kHz


#endif /* _PROCESSOR_DEFS_H */