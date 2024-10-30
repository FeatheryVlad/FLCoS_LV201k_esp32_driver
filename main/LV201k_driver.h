
#ifndef _LV201K_DEFS_H
#define _LV201K_DEFS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_err.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

typedef struct {
  uint8_t cmd;
  uint8_t data;
  uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lvd_cmd_t;

DRAM_ATTR static const lvd_cmd_t preset_lvd_regs[] = {
  /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
  //{0x00, 0x01, 1},
  {0x00, 0x01, 1},
  //{0x01, 0x10, 1},
  {0x01, 0x10, 1},
  //{0x02, 0x00, 1},
  {0x02, 0x00, 1},
  //{0x03, 0xFF, 1},
  {0x03, 0xFF, 1},
  //{0x04, 0x08, 1},
  {0x04, 0x08, 1},
  //{0x05, 0x00, 1},
  {0x05, 0x00, 1},
  //{0x06, 0x48, 1},
  {0x06, 0x48, 1},
  //{0x07, 0x04, 1},
  //{0x08, 0xD6, 1},
  //{0x09, 0x05, 1},
  //{0x0a, 0xDE, 1},
  //{0x0b, 0x0E, 1},
  //{0x0e, 0x09, 1},

  //{0x07, flreg07, 1},
  //{0x08, flreg08, 1},
  //{0x09, flreg09, 1},
  //{0x0a, flreg0a, 1},
  //{0x0b, flreg0b, 1},

  //{0x0c, 0x01, 1},
  //{0x0d, 0x00, 1},

  {0x0e, 0x00, 1},
  //{0x0f, {0x01}, 1},
  //{0x10, {0x01}, 1},
  ///* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
  //{0x3A, {0x55}, 1},
  ///* Porch Setting */
  //{0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
  ///* Gate Control, Vgh=13.65V, Vgl=-10.43V */
  //{0xB7, {0x45}, 1},
  ///* VCOM Setting, VCOM=1.175V */
  //{0xBB, {0x2B}, 1},
  ///* LCM Control, XOR: BGR, MX, MH */
  //{0xC0, {0x2C}, 1},
  ///* VDV and VRH Command Enable, enable=1 */
  //{0xC2, {0x01, 0xff}, 2},
  ///* VRH Set, Vap=4.4+... */
  //{0xC3, {0x11}, 1},
  ///* VDV Set, VDV=0 */
  //{0xC4, {0x20}, 1},
  ///* Frame Rate Control, 60Hz, inversion=0 */
  //{0xC6, {0x0f}, 1},
  ///* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
  //{0xD0, {0xA4, 0xA1}, 2},
  ///* Positive Voltage Gamma Control */
  //{0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
  ///* Negative Voltage Gamma Control */
  //{0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
  ///* Sleep Out */
  //{0x11, {0}, 0x80},
  ///* Display On */
  //{0x29, {0}, 0x80},
  {0, 0, 0xff}
};

//SPI
esp_err_t lvd_init_spi(int spiCLK, int spiMOSI, int spiMISO, int csPin);
esp_err_t lvd_SPI_write(uint8_t address, uint8_t sendData, int csPin);
esp_err_t lvd_SPI_read(uint8_t address, int csPin);

void lvd_initialize(int spiCLK, int spiMOSI, int spiMISO);

#endif /* _LV201K_DEFS_H */