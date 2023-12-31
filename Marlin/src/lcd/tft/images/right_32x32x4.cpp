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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_GRAPHICAL_TFT
extern const uint8_t right_round_36x36x4[648] = {
0X00,0X00,0X00,0X00,0X00,0X00,0X26,0X9B,0XEF,0XFF,0XB8,0X50,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X5B,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X93,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X3D,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XB1,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X09,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFE,0X60,0X00,0X00,0X00,0X00,0X00,0X00,0XCF,0XFF,0XFF,0XFF,0XFD,
0XC8,0X9C,0XEF,0XFF,0XFF,0XFF,0XF9,0X00,0X00,0X01,0X00,0X00,0X0C,0XFF,0XFF,0XFF,
0XC4,0X00,0X00,0X00,0X01,0X7E,0XFF,0XFF,0XFF,0X90,0X00,0X6C,0X00,0X00,0XCF,0XFF,
0XFF,0XD3,0X00,0X00,0X00,0X00,0X00,0X00,0X6F,0XFF,0XFF,0XF9,0X2B,0XF9,0X00,0X09,
0XFF,0XFF,0XFA,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0XDF,0XFF,0XFF,0XFF,0XF4,
0X00,0X4F,0XFF,0XFF,0X70,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0D,0XFF,0XFF,
0XFF,0XF0,0X00,0XCF,0XFF,0XFA,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,
0XEF,0XFF,0XFF,0XD0,0X03,0XFF,0XFF,0XD0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X3D,0XFF,0XFF,0XFF,0X90,0X0B,0XFF,0XFF,0X30,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X0A,0XFF,0XFF,0XFF,0XFF,0X40,0X0F,0XFF,0XFC,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X04,0XAE,0XFF,0XFF,0XFF,0X10,0X6F,0XFF,0XF6,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X28,0XDF,0XFE,0X00,0XAF,0XFF,0XF0,0X00,
0X00,0X00,0X00,0X01,0XAD,0XDA,0X10,0X00,0X00,0X00,0X00,0X01,0X67,0X00,0XCF,0XFF,
0XC0,0X00,0X00,0X00,0X00,0X1E,0XFF,0XFF,0XE1,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0XFF,0XFF,0XA0,0X00,0X00,0X00,0X00,0XBF,0XFF,0XFF,0XFB,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFD,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFE,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XA0,0X00,0X00,0X00,0X00,0XBF,0XFF,0XFF,
0XFB,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XCF,0XFF,0XC0,0X00,0X00,0X00,0X00,0X1F,
0XFF,0XFF,0XF1,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XAF,0XFF,0XF0,0X00,0X00,0X00,
0X00,0X01,0XBE,0XEB,0X10,0X00,0X00,0X00,0X00,0X4D,0X94,0X00,0X6F,0XFF,0XF4,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XAF,0XFF,0XF1,0X1F,0XFF,
0XFB,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XEF,0XFF,0XD0,
0X0C,0XFF,0XFF,0X30,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X08,0XFF,
0XFF,0X70,0X06,0XFF,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X2F,0XFF,0XFF,0X00,0X00,0XDF,0XFF,0XF9,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XCF,0XFF,0XF8,0X00,0X00,0X3F,0XFF,0XFF,0X80,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X0C,0XFF,0XFF,0XE0,0X00,0X00,0X0A,0XFF,0XFF,0XF9,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X01,0XCF,0XFF,0XFF,0X40,0X00,0X00,0X00,0XDF,0XFF,0XFF,0XD3,
0X00,0X00,0X00,0X00,0X00,0X00,0X6E,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X00,0X0D,0XFF,
0XFF,0XFF,0XB5,0X00,0X00,0X00,0X01,0X6D,0XFF,0XFF,0XFF,0XA0,0X00,0X00,0X00,0X00,
0X00,0XDF,0XFF,0XFF,0XFF,0XFC,0XB7,0X9B,0XDF,0XFF,0XFF,0XFF,0XF9,0X00,0X00,0X00,
0X00,0X00,0X00,0X0A,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X60,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X3E,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XB1,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X6D,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XA3,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X17,0XAD,0XFF,0XFE,
0XC9,0X40,0X00,0X00,0X00,0X00,0X00,0X00,
};


extern const uint8_t right_32x32x4[512] = {
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x77, 0x77, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x77, 0x8F, 0xA7, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x87, 0x78, 0xFF, 0xF9, 0x77, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x87, 0x8F, 0xFF, 0xFF, 0x97, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x87, 0x7B, 0xFF, 0xFF, 0xF9, 0x77, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x84, 0xAF, 0xFF, 0xFF, 0x97, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x3A, 0xFF, 0xFF, 0xF9, 0x77, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x83, 0xAF, 0xFF, 0xFF, 0x97, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x3A, 0xFF, 0xFF, 0xF9, 0x77, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x83, 0xAF, 0xFF, 0xFF, 0x97, 0x78, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x4A, 0xFF, 0xFF, 0xF9, 0x77, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x74, 0x9F, 0xFF, 0xFF, 0x97, 0x78, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x87, 0x49, 0xFF, 0xFF, 0xFA, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x76, 0xFF, 0xFF, 0xFC, 0x67, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x77, 0x9F, 0xFF, 0xFF, 0xC4, 0x36, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x87, 0x79, 0xFF, 0xFF, 0xFC, 0x33, 0x46, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x77, 0x9F, 0xFF, 0xFF, 0xC3, 0x34, 0x78, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x87, 0x79, 0xFF, 0xFF, 0xFD, 0x33, 0x47, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x87, 0x8F, 0xFF, 0xFF, 0xD3, 0x44, 0x78, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x78, 0xFF, 0xFF, 0xFC, 0x34, 0x47, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x77, 0x8F, 0xFF, 0xFF, 0xC3, 0x44, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x87, 0x78, 0xFF, 0xFF, 0xFC, 0x33, 0x47, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x87, 0x9F, 0xFF, 0xFF, 0xC4, 0x34, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x87, 0x8A, 0xFF, 0xFC, 0x43, 0x47, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x84, 0x8F, 0xC4, 0x34, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x4A, 0x53, 0x47, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x84, 0x34, 0x78, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x87, 0x56, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
  0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88
};

#endif // HAS_GRAPHICAL_TFT
