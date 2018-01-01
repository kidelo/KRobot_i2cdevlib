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
// Authors:
// Jeff Rowberg   - I2Cdev device library code and interface
// Kidelo         - This complete new implementation for Arduino / ATMEGA
//
// **************************************************************************************

// force optimize for speed
#pragma GCC optimize ("O3")

// IMPORT
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// guard build without feed of arduino environment
// #undef ARDUINO

// optionally ( activates debug log and microsecond timeouts )
#ifdef ARDUINO
  #include <arduino.h>
#endif

// EXPORT
#include <I2Cdev.h>

//** DEFINES ******************************************************************

//** DEFINES ******************************************************************

#ifdef ARDUINO
// i2c_wait_us -> 100 bytes opcode more
//#  define i2c_wait i2c_wait_us
#  define i2c_wait i2c_wait_cnt
#else
#  define i2c_wait i2c_wait_cnt
#endif

//** CONSTANTS ****************************************************************

// TWSR status code ( valid for prescaler = 0 )
enum
{
  // TWI Master status codes
  I2C_START		    = 0x08,  // A START condition has been transmitted
  I2C_REP_START	    = 0x10,  // A repeated START condition has been transmitted
  I2C_ARB_LOST	    = 0x38,  // Arbitration lost in SLA+W or data bytes

  // TWI Master Transmitter status codes -> data from MASTER to SLAVE
  I2C_TX_ADR_ACK	= 0x18,  // SLA+W has been transmitted; ACK has been received
  I2C_TX_ADR_NACK   = 0x20,  // SLA+W has been transmitted; NOT ACK has been received
  I2C_TX_DATA_ACK 	= 0x28,  // Data byte has been transmitted; ACK has been received
  I2C_TX_DATA_NACK	= 0x30,  // Data byte has been transmitted; NOT ACK has been received

  // TWI Master Receiver status codes -> data from SLAVE to MASTER
  I2C_RX_ADR_ACK	= 0x40,  // SLA+R has been transmitted; ACK has been received
  I2C_RX_ADR_NACK   = 0x48,  // SLA+R has been transmitted; NOT ACK has been received
  I2C_RX_DATA_ACK	= 0x50,  // Data byte has been received; ACK has been returned
  I2C_RX_DATA_NACK	= 0x58,  // Data byte has been received; NOT ACK has been returned
};

/** CONSTANTS ***************************************************************/

enum
{
  // Frequency = 100kHz
  I2C_FREQ             = 100000,

  // mask out lower bits of prescaler
  I2C_PRESCALER_MASK   = 0xF8,

  // wait for response [µs], .. I2C SLAVE may stretch clock !
  I2C_DEV_RESP_WAIT_US = 50
};

/** DEFINES *******************************************************************/

// TWCR – TWI Control Register
// -----------------------------------------
// Bit 7 – TWINT: TWI Interrupt Flag
// Bit 6 – TWEA:  TWI Enable Acknowledge Bit
// Bit 5 – TWSTA: TWI START Condition Bit
// Bit 4 – TWSTO: TWI STOP Condition Bit
// Bit 3 – TWWC:  TWI Write Collision Flag
// Bit 2 – TWEN:  TWI Enable Bit
// Bit 1 – Res:   Reserved Bit
// Bit 0 – TWIE:  TWI Interrupt Enable

//** FUNCTION ******************************************************************

void i2c_drv_init()
{
  // set bitrate
  TWBR = ((F_CPU/I2C_FREQ)-16)/(2*4^0 /*TWPS*/ );
  TWBR = 1;

  // cleanup prescaler = 0 -> 1
  TWSR = 0;

  // enable pullups
  PORTD |= ((1 << 0) | (1 << 1));

  // generate stop condition
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

} // end of i2c_drv_init()

//** FUNCTION ******************************************************************

// check status of last TWI action
inline bool i2c_status( uint8_t mask )
{
  return ( ((TWSR & I2C_PRESCALER_MASK) == mask ) ? true : false );

} // end of i2c_status()

//** FUNCTION ******************************************************************

// wait for end of last triggered action
inline bool i2c_wait_cnt()
{
  // default timeout
  uint8_t uI2cWait = I2C_DEV_RESP_WAIT_US;

  // poll ISR status register
  while ( !(TWCR & (1<<TWINT)) && uI2cWait-- )
  {
    // spend time
    delayMicroseconds( 1 ); yield();
  }

  // check it
  return uI2cWait > 0;

} // end of i2c_wait()

//** FUNCTION ******************************************************************

#ifdef ARDUINO

// wait for end of last triggered action
bool i2c_wait_us()
{
  // start entry
  const uint32_t start = micros();
  const uint16_t waitUs = 100;

  bool wait = true;

  // check and wait if necessary
  while ( !(TWCR & (1<<TWINT)) && wait );
  {
    wait = (uint16_t)( micros() - start ) < waitUs;
  }

  return wait;

} // end of i2c_wait_us()

#endif

//** FUNCTION ******************************************************************

// generate a stop condition and release the TWI interface
inline void i2c_drv_stop()
{
  // generate stop condition
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

  // wait -> result don't care
  i2c_wait();

  // release the bus -> IDLE
  TWCR = (1<<TWINT);

} // end of i2c_drv_stop()

//** FUNCTION ******************************************************************

// selection I2C device for R=0 / W=1
inline bool i2c_drv_sel_dev_s( uint8_t devAddr, bool write )
{
  bool bOk;

  // generate a start condition on the bus
  TWCR = ( (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1 << TWSTO));

  // wait for end of
  bOk = i2c_wait() && ( i2c_status( I2C_START ) || i2c_status( I2C_REP_START ) );

  if( bOk )
  {
    // write: send device address + 'R/_W = 0'
    TWDR = (devAddr<<1) | ( write ? 0 : 1 );

    // start transmission
    TWCR = (1<<TWINT)|(1<<TWEN);

    // select flag by action
    const uint8_t uFlag = write ? I2C_TX_ADR_ACK : I2C_RX_ADR_ACK;

    // wait for end of transmission and check for ACK
    bOk = i2c_wait() && i2c_status( uFlag );
  }

  return bOk;

} // end of i2c_sel_dev()

//** FUNCTION ******************************************************************

inline bool i2c_drv_sel_dev( uint8_t devAddr, bool write, uint8_t trials = 3 )
{
  bool bOk = false;

  do
  {
    // try it
    bOk = i2c_drv_sel_dev_s( devAddr, write );

    // on error ->
    if ( !bOk )
    {
      i2c_drv_stop();
    }

  } while( !bOk && --trials );

  return bOk;
}

//** FUNCTION ******************************************************************

// transfer a byte from MASTER to current selected SLAVE
inline bool i2c_drv_wr( uint8_t data )
{
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);

  // wait and check for status
  return i2c_wait() && i2c_status( I2C_TX_DATA_ACK );
}

//** FUNCTION ******************************************************************

// write address of a virtual register to I2C device
inline bool i2c_drv_wr_addr( uint16_t devAddr, uint8_t len )
{
  bool bOk = false;

  // for 2byte addresses
  if ( 2 == len )
  {
    bOk = i2c_drv_wr( (uint8_t)( devAddr >> 8 ) & 0xFF );
  }

  if ( bOk )
  {
    bOk = i2c_drv_wr( (uint8_t)( devAddr & 0xFF ) & 0xFF );
  }

  return bOk;

} // end of i2c_wr_addr()

//** FUNCTION ******************************************************************

// Read a register from device = uMemAddr
bool i2c_drv_read_reg_m( uint8_t devAddr, uint8_t addrBytes, uint16_t regAddr, uint8_t * pData, uint16_t uSize )
{
  bool bOk = true;

  // release the bus -> IDLE
  TWCR = (1<<TWINT);

  // select device for WRITE (first start)
  bOk = i2c_drv_sel_dev( devAddr, true );

  // write register address to I2C device
  bOk = bOk && i2c_drv_wr_addr( regAddr, addrBytes );

  // select device for READ (repeated start)
  bOk = bOk && i2c_drv_sel_dev( devAddr, false );

  // for all requested bytes
  while( bOk && uSize-- )
  {
    //MASTER will request more bytes
    const bool bAck = ( 0 != uSize );

    // request next byte and send a ACK (always ... )
    TWCR = (1 << TWINT) | (1 << TWEN) | ( ( bAck ? 1 : 0 ) << TWEA);

    // wait for transfer
    bOk = i2c_wait() && i2c_status( bAck ? I2C_RX_DATA_ACK : I2C_RX_DATA_NACK );

    // read out data ( may be garbage ... )
    const uint8_t data = TWDR;

    // return to caller
    if ( bOk )
    {
      *(pData)++ = data;
    }
  }

  // stop condition
  i2c_drv_stop();

  return bOk;

} // end of i2c_drv_read_reg_m()

//** FUNCTION ******************************************************************

// Read a single register from device
uint8_t i2c_drv_read_reg_s( uint8_t devAddr, uint8_t regAddr )
{
  uint8_t val;

  // request it
  const bool bOk = i2c_drv_read_reg_m( devAddr, 1, regAddr, &val, 1 );

  // on error return default value
  return bOk ? val : 0xFF;
}

//** FUNCTION ******************************************************************

bool i2c_drv_write_reg_m( uint8_t devAddr, uint8_t addrBytes, uint16_t regAddr, const uint8_t * pData, uint16_t uSize )
{
  bool bOk = true;

  // release the bus -> IDLE
  TWCR = (1<<TWINT);

  // select device for WRITE (first start)
  bOk = i2c_drv_sel_dev( devAddr, true );

  // write register address to I2C device
  bOk = bOk && i2c_drv_wr_addr( regAddr, addrBytes );

  // for all requested bytes
  while( bOk && uSize-- )
  {
    // get next byte
    const uint8_t data = *(pData)++;

    // store prepare for transport
    TWDR = data;

    // request next transfer from MASTER to SAVE and wait for a ACK
    TWCR = (1 << TWINT) | (1 << TWEN);

    // wait for end of transfer
    bOk = i2c_wait() && i2c_status( I2C_TX_DATA_ACK );
  }

  // stop condition
  i2c_drv_stop();

  return bOk;

} // end of i2c_drv_write_reg_m()

//** FUNCTION ******************************************************************

// write to a single register of device
bool i2c_drv_write_reg_s( uint8_t devAddr, uint8_t regAddr, uint8_t val )
{
  // request it
  return i2c_drv_write_reg_m( devAddr, 1, regAddr, &val, 1 );
}

//** FUNCTION ******************************************************************

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
  uint8_t b=0;
  const bool ok = readByte(devAddr, regAddr, &b);

  if (ok) {
    *data = b & (1 << bitNum);
  }

  return ok;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data) {
  uint16_t b=0;
  const bool ok = readWord(devAddr, regAddr, &b);

  if (ok) {
    *data = b & (1 << bitNum);
  }

  return ok;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t b = 0;
  const bool ok = readByte(devAddr, regAddr, &b);

  if (ok) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return ok;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
bool I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data) {
  // 1101011001101001 read byte
  // fedcba9876543210 bit numbers
  //    xxx           args: bitStart=12, length=3
  //    010           masked
  //           -> 010 shifted
  uint16_t w = 0;
  const bool ok = readWord(devAddr, regAddr, &w);
  if (ok) {
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    w &= mask;
    w >>= (bitStart - length + 1);
    *data = w;
  }
  return ok;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
  return i2c_drv_read_reg_m( devAddr, 1, regAddr, (uint8_t*)data, sizeof(*data) );
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
  return i2c_drv_read_reg_m( devAddr, 1, regAddr, (uint8_t*)data, sizeof(*data) );
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
  return i2c_drv_read_reg_m( devAddr, 1, regAddr, (uint8_t*)data, length * sizeof(*data) );
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {
  return i2c_drv_read_reg_m( devAddr, 1, regAddr, (uint8_t*)data, length * sizeof(*data) );
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
  uint8_t b;
  bool ok = readByte(devAddr, regAddr, &b);

  if (ok) {
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    ok = writeByte(devAddr, regAddr, b);
  }
  return ok;
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
  uint16_t w = 0;
  bool ok = readWord(devAddr, regAddr, &w);

  if (ok) {
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    ok = writeWord(devAddr, regAddr, w);
  }
  return ok;
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b;
  bool ok = readByte(devAddr, regAddr, &b);

  if (ok) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    ok = writeByte(devAddr, regAddr, b);
  }

  return ok;
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
  //              010 value to write
  // fedcba9876543210 bit numbers
  //    xxx           args: bitStart=12, length=3
  // 0001110000000000 mask word
  // 1010111110010110 original value (sample)
  // 1010001110010110 original & ~mask
  // 1010101110010110 masked | value
  uint16_t w;
  bool ok = readWord(devAddr, regAddr, &w);

  if (ok) {
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    w &= ~(mask); // zero all important bits in existing word
    w |= data; // combine data with existing word
    ok = writeWord(devAddr, regAddr, w);
  }
  return ok;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
  return i2c_drv_write_reg_m( devAddr, 1, regAddr, &data, sizeof(data) );
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
  return i2c_drv_write_reg_m( devAddr, 1, regAddr, (uint8_t*)&data, sizeof(data) );
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) {
  return i2c_drv_write_reg_m( devAddr, 1, regAddr, (const uint8_t*)data, length * sizeof(data) );
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint16_t* data) {
  return i2c_drv_write_reg_m( devAddr, 1, regAddr, (const uint8_t*)data, length * sizeof(data) );
}
