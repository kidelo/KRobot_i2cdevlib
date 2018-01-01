// **************************************************************************************
//
// Copyright (c) 2017      Kidelo       <kidelo@yahoo.com>
// Copyright (c) 2011-2015 Jeff Rowberg <jeff@rowberg.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// **************************************************************************************
//
// AUTHORS:
// Jeff Rowberg   - I2Cdev device library code and interface
// Kidelo         - This complete new implementation for Arduino / ATMEGA
//
// CHANGELOG:
// 2018-01-01 - kidelo : adapt to new I2C master / cleanup / fix issues ...
//
// ... original...
//
// 2015-10-30 - simondlevy : support i2c_t3 for Teensy3.1
// 2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
// 2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
// 2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//            - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
// 2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
// 2011-10-03 - added automatic Arduino version detection for ease of use
// 2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
// 2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
// 2011-08-03 - added optional timeout parameter to read* methods to easily change from default
// 2011-08-02 - added support for 16-bit registers
//            - fixed incorrect Doxygen comments on some methods
//            - added timeout value for read operations (thanks mem @ Arduino forums)
// 2011-07-30 - changed read/write function structures to return success or byte counts
//            - made all methods static for multi-device memory savings
// 2011-07-28 - initial release

// **************************************************************************************

#ifndef _I2C_DEV_LIGHT_H_
#define _I2C_DEV_LIGHT_H_

// IMPORT
#include <stdint.h>

// FUNCTIONS ****************************************************************************

// init the I2C peripheral
void    i2c_drv_init();

// read a range of 1..n 8 bit registers starting from uMemAddr from uI2cAddr
bool    i2c_drv_read_reg_m( uint8_t devAddr, uint8_t addrBytes, uint16_t regAddr, uint8_t * pData, uint16_t uSize );
uint8_t i2c_drv_read_reg_s( uint8_t devAddr, uint8_t regAddr );

// write a range of 1..n 8 bit registers starting from uMemAddr to uI2cAddr
bool    i2c_drv_write_reg_m( uint8_t devAddr, uint8_t addrBytes, uint16_t regAddr, const uint8_t * pData, uint16_t uSize );
bool    i2c_drv_write_reg_s( uint8_t devAddr, uint8_t regAddr, uint8_t val );

// **************************************************************************************

// interface based on with modifications, timeout handling is now part of base functions
//
// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
//
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
//

// only a namespace
struct I2Cdev
{
  // read interface, see implementation for interface description
  static bool readBit  (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
  static bool readBitW (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
  static bool readBits (uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
  static bool readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
  static bool readByte (uint8_t devAddr, uint8_t regAddr, uint8_t *data);
  static bool readWord (uint8_t devAddr, uint8_t regAddr, uint16_t *data);
  static bool readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
  static bool readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

  // write interface, , see implementation for interface description
  static bool writeBit  (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
  static bool writeBitW (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
  static bool writeBits (uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
  static bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
  static bool writeByte (uint8_t devAddr, uint8_t regAddr, uint8_t data);
  static bool writeWord (uint8_t devAddr, uint8_t regAddr, uint16_t data);
  static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *data);
  static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint16_t *data);

private:

  // guard
  ~I2Cdev();
};

#endif // end of _I2C_DEV_LIGHT_H_
