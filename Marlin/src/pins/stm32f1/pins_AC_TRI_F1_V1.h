/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#if NOT_TARGET(STM32F1)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#endif

//#define TFT_LVGL_UI_SPI
//#define HAS_RESUME_CONTINUE 1

#define BOARD_INFO_NAME      "AC_TRI_F1"
#define DEFAULT_MACHINE_NAME "AnycubicKobra"

#define BOARD_NO_NATIVE_USB
#define LEVEING_CALIBRATION_MODULE
#define SD_CARD_LOG
//#define DISABLE_JTAG
//#define DISABLE_DEBUG

//
// EEPROM
//
#define FLASH_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE              0x1000  // 4KB

#if 0
#if ENABLED(FLASH_EEPROM_EMULATION)
  // SoC Flash (framework-arduinoststm32-maple/STM32F1/libraries/EEPROM/EEPROM.h)
  #define EEPROM_START_ADDRESS (0x8000000UL + (512 * 1024) - 2 * EEPROM_PAGE_SIZE)
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB, but will use 2x more (4KB)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE
#else
  #define MARLIN_EEPROM_SIZE              0x800U  // On SD, Limit to 2KB, require this amount of RAM
#endif
#endif

//
// Limit Switches
//
#if 0
#define X_STOP_PIN                          PA7
#define Y_STOP_PIN                          PC5
#define ZL_STOP_PIN                         PB2
#define ZR_STOP_PIN                         PC6
#define Z_STOP_PIN                          ZL_STOP_PIN
#endif


//
// Steppers
//
#define X_ENABLE_PIN                        PA15
#define X_STEP_PIN                          PA12
#define X_DIR_PIN                           PA11
#define X_STALL_PIN                         -1

#define Y_ENABLE_PIN                        X_ENABLE_PIN
#define Y_STEP_PIN                          PA9
#define Y_DIR_PIN                           PA8
#define Y_STALL_PIN                         -1

#define Z_ENABLE_PIN                        X_ENABLE_PIN
#define Z_STEP_PIN                          PB0
#define Z_DIR_PIN                           PB1
#define Z_STALL_PIN                         -1

#define X2_ENABLE_PIN                       X_ENABLE_PIN
#define X2_STEP_PIN                         -1
#define X2_DIR_PIN                          -1
#define X2_STALL_PIN                        -1



#define Y2_ENABLE_PIN                       X_ENABLE_PIN
#define Y2_STEP_PIN                         -1//PC7
#define Y2_DIR_PIN                          -1//PC6
#define Y2_STALL_PIN                        -1


// #define Z_ENABLE_PIN                        X_ENABLE_PIN
// #define Z_STEP_PIN                          PC7
// #define Z_DIR_PIN                           PC6
// #define Z_STALL_PIN                         -1

#define Z2_ENABLE_PIN                       X_ENABLE_PIN
#define Z2_STEP_PIN                         -1
#define Z2_DIR_PIN                          -1
#define Z2_STALL_PIN                        -1

#define E0_ENABLE_PIN                       X_ENABLE_PIN
#define E0_STEP_PIN                         PB15
#define E0_DIR_PIN                          PB14
#define E0_STALL_PIN                        -1




//
// Steppers
//
#define ONBOARD_ENDSTOPPULLUPS              // this remove #error "SENSORLESS_HOMING requires ENDSTOPPULLUP_XMIN

#define X_MIN_PIN                           PB11
#define X_MAX_PIN                           X2_STALL_PIN
//#define X2_MIN_PIN                          X2_STALL_PIN
//#define X2_MAX_PIN                          -1

#define Y_MIN_PIN                           PC13
#define Y_MAX_PIN                           Y2_STALL_PIN
//#define Y2_MIN_PIN                          Y2_STALL_PIN
//#define Y2_MAX_PIN                          -1

#define Z_MIN_PIN                           PA1
#define Z_MAX_PIN                           PA0
//#define Z2_MIN_PIN                          PA11
//#define Z2_MAX_PIN                          -1

#define E0_MIN_PIN                          -1


#if 0 //HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
#define X2_HARDWARE_SERIAL  Serial1
#define Y_HARDWARE_SERIAL   Serial1
#define Y2_HARDWARE_SERIAL  Serial1
#define Z2_HARDWARE_SERIAL  Serial1

#define X_SERIAL_TX_PIN     PB9
#define X_SERIAL_RX_PIN     PB9
#define Z_SERIAL_TX_PIN    X_SERIAL_TX_PIN
#define Z_SERIAL_RX_PIN    X_SERIAL_RX_PIN
#define E0_SERIAL_TX_PIN   X_SERIAL_TX_PIN
#define E0_SERIAL_RX_PIN   X_SERIAL_RX_PIN

//#define Z_HARDWARE_SERIAL   Serial1
//#define Z2_HARDWARE_SERIAL  Serial1
//#define E0_HARDWARE_SERIAL  Serial1


//
// Software serial
//
//  #define X_SERIAL_TX_PIN                   PA9
//  #define X_SERIAL_RX_PIN                   PA10

//  #define Y_SERIAL_TX_PIN                   PA9
//  #define Y_SERIAL_RX_PIN                   PA10

//  #define Z_SERIAL_TX_PIN                   PA9
//  #define Z_SERIAL_RX_PIN                   PA10

//  #define E0_SERIAL_TX_PIN                  PA9
//  #define E0_SERIAL_RX_PIN                  PA10

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                     19200
#endif



//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC3
#define TEMP_1_PIN                          -1    // T1
#define TEMP_BED_PIN                        PC1   // TB

//
// Heaters
//
#define HEATER_0_PIN                        PB8
#define HEATER_1_PIN                        -1
#define HEATER_BED_PIN                      PB9
#define POWER_CTRL_PIN                      PB6

//
// Fans
//
#define FAN_PIN                             PB5  // FAN
#define FAN1_PIN                            PB13  // // SWCLK
#define FAN2_PIN                            PB12   // PB3, auto fan for E0
#define CONTROLLER_FAN_PIN                  FAN2_PIN

//
// Misc
//
#define BEEPER_PIN                          PB7
#define FIL_RUNOUT_PIN                      PC15
#define LED_PIN                             -1
#define CASE_LIGHT_PIN                      -1
#define POWER_LOSS_PIN                      PC2
#define POWER_MONITOR_VOLTAGE_PIN           PC2


#define AUTO_LEVEL_RX_PIN                   PB8

//
// SD Card
//
#define SDIO_SUPPORT
#define SD_DETECT_PIN                       PA10

#ifdef SDIO_SUPPORT
//
// SPI
//
  #define SPI_DEVICE                          -1
  #define SCK_PIN                             -1
  #define MISO_PIN                            -1
  #define MOSI_PIN                            -1
  #define SS_PIN                              -1

//
// SDIO
//
  #define SDIO_READ_RETRIES                   16
  #define SDIO_D0_PIN                         PC8
  #define SDIO_D1_PIN                         PC9
  #define SDIO_D2_PIN                         PC10
  #define SDIO_D3_PIN                         PC11
  #define SDIO_CK_PIN                         PC12
  #define SDIO_CMD_PIN                        PD2

#else

  #undef  SDSS
  #define SOFTWARE_SPI
  #define SS_PIN                   PC11
  #define SDSS                     PC11 // SDIO_D3_PIN
  #define SCK_PIN                  PC12 // SDIO_CK_PIN
  #define MISO_PIN                 PC8  // SDIO_D0_PIN
  #define MOSI_PIN                 PD2  // SDIO_CMD_PIN

#endif
// TFT SPI CONFIG//
    #define TFT_BACKLIGHT_PIN        PC0
    #define TFT_CS_PIN               PA4
    #define TFT_DC_PIN               PA6
    #define TFT_A0_PIN               TFT_DC_PIN
    #define TFT_SCK_PIN              PA5
    #define TFT_MOSI_PIN             PA7
    #define TFT_MISO_PIN             TFT_MOSI_PIN

    #define BTN_ENC                 PB4
    #define BTN_EN1                 PB10
    #define BTN_EN2                 PB3