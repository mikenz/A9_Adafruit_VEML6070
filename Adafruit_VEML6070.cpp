/*!
 * @file Adafruit_VEML6070.cpp
 *
 * @mainpage Adafruit VEML6070 UV sensor
 *
 * @section intro_sec Introduction
 *
 * Designed specifically to work with the VEML6070 sensor from Adafruit
 * ----> https://www.adafruit.com/products/2899
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_VEML6070.h"
#include "api_debug.h"
#include "api_os.h"

/**************************************************************************/
/*!
    @brief constructor initializes default configuration value
*/
/**************************************************************************/
Adafruit_VEML6070::Adafruit_VEML6070() {
    //default setting
    _commandRegister.reg = 0x02;
}

/**************************************************************************/
/*!
    @brief  setup and initialize communication with the hardware
    @param itime the integration time to use for the data
    @param twoWire Optional pointer to the desired TwoWire I2C object. Defaults to &Wire
*/
/**************************************************************************/
void Adafruit_VEML6070::begin(I2C_ID_t i2c, veml6070_integrationtime_t itime)
{
  _i2c = i2c;
  _i2cConfig.freq = I2C_FREQ_100K;
  I2C_Init(_i2c, _i2cConfig);

  _commandRegister.bit.IT = itime;

  clearAck();
  writeCommand();
}

/**************************************************************************/
/*!
    @brief  Set the threshold-based interrupt feature
    @param  state true to enable, false to disable
    @param  level 1 for threshold value of 145, 0 for 102 (default)
*/
/**************************************************************************/
void Adafruit_VEML6070::setInterrupt(bool state, bool level) {
  _commandRegister.bit.ACK = state;
  _commandRegister.bit.ACK_THD = level;

  clearAck();
  writeCommand();
}

/**************************************************************************/
/*!
    @brief  Clear possible interrupt state (ACK active) by reading register
            If set, MUST be cleared before device will respond at other
            I2C addresses.
            See datasheet rev 1.7, p.7, and app note p. 13 (example code)
    @return True if ACK was active (interrupt triggered)
*/
/**************************************************************************/
bool Adafruit_VEML6070::clearAck() {
  uint8_t res = 0;
  I2C_Error_t error = I2C_Receive(_i2c, VEML6070_ADDR_ARA, &res, 1, I2C_DEFAULT_TIME_OUT);
  if (error != I2C_ERROR_NONE)
  {
    Trace(1, "Adafruit_VEML6070::clearAck recieve error: 0X%02x", error);
  }
  return res;
}

/**************************************************************************/
/*!
    @brief  read the chips UV sensor
    @return the UV reading as a 16 bit integer
*/
/**************************************************************************/
uint16_t Adafruit_VEML6070::readUV() {
  waitForNext();

  uint8_t highByte = 0;
  uint8_t lowByte = 0;
  I2C_Error_t error = I2C_Receive(_i2c, VEML6070_ADDR_H, &highByte, 1, I2C_DEFAULT_TIME_OUT);
  if (error != I2C_ERROR_NONE)
  {
    Trace(1, "Adafruit_VEML6070::readUV recieve error: 0X%02x", error);
    return -1;
  }
  error = I2C_Receive(_i2c, VEML6070_ADDR_L, &lowByte, 1, I2C_DEFAULT_TIME_OUT);
  if (error != I2C_ERROR_NONE)
  {
    Trace(1, "Adafruit_VEML6070::readUV recieve error: 0X%02x", error);
    return -1;
  }

  return (highByte << 8) | lowByte;
}

/**************************************************************************/
/*!
    @brief  wait for one integration period (with ~10% clock error margin)
*/
/**************************************************************************/
void Adafruit_VEML6070::waitForNext() {
  // Map the integration time code to the correct multiple (datasheet p. 8)
  // {0 -> 1, 1 -> 2; 2 -> 4; 3 -> 8}
  uint8_t itCount = 1;
  for (uint8_t i = _commandRegister.bit.IT; i > 0; i--) { itCount *= 2; }

  for (uint8_t i = 0; i < itCount; i++) {
    OS_Sleep(63);  // Depends on RSET = 270K, note actual time is shorter
                   // than 62.5ms for RSET = 300K in datasheet table
  }
}

/**************************************************************************/
/*!
    @brief  enter or exit sleep (shutdown) mode. While in sleep mode
      the chip draws ~1uA
    @param state true to enter sleep mode, false to exit
*/
/**************************************************************************/
void Adafruit_VEML6070::sleep(bool state) {
  _commandRegister.bit.SD = state;

  writeCommand();
}


/**************************************************************************/
/*!
    @brief write current internal _commandRegister value to device
*/
/**************************************************************************/
void Adafruit_VEML6070::writeCommand() {
  I2C_Error_t error = I2C_Transmit(_i2c, VEML6070_ADDR_L, &_commandRegister.reg, 1, I2C_DEFAULT_TIME_OUT);
  if (error != I2C_ERROR_NONE)
  {
    Trace(1, "Adafruit_VEML6070::writeCommand transmit error: 0X%02x", error);
  }
}
