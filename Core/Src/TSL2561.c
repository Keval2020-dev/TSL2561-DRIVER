/*!
 * @file Adafruit_TSL2561_U.c
 *
 * This is part of Adafruit's FXOS8700 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit FXOS8700 breakout:
 * https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "main.h"
#include "TSL2561.h"

extern uint8_t Reg_Buf[];  //defined in main file
extern int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length); //defined in main file
extern int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);  //defined in main file
extern int8_t i2c_only_reg_write(uint8_t i2c_addr, uint8_t reg_addr);

/**************************************************************************/
/*!
    @brief Configuration Setup of sensor
    @param addr The I2C address this chip can be found on, 0x29, 0x39 or 0x49
    @param sensorID An optional ID that will be placed in sensor events to help
                    keep track if you have many sensors in use
*/
/**************************************************************************/
void TSL2561_Config(uint8_t addr, int32_t sensorID)
{
  _addr = addr;
  _tsl2561Initialised = 0;
  _tsl2561AutoGain = 0;
  _tsl2561IntegrationTime = TSL2561_INTEGRATIONTIME_13MS;
  _tsl2561Gain = TSL2561_GAIN_1X;
  _tsl2561SensorID = sensorID;
}



/**************************************************************************/
/*!
    @brief  Initializes I2C connection and settings.
    Attempts to determine if the sensor is present on the i2c bus, then sets up a default
    integration time and gain. Then powers down the chip.
    @returns True if sensor is found and initialized, false otherwise.
*/
/**************************************************************************/
_Bool TSL2561_Init()
{
  /* Make sure we're actually connected */

  i2c_reg_read(_addr,TSL2561_REGISTER_ID,Reg_Buf,1);
  uint8_t x = Reg_Buf[0];

  if (x & 0x05) { // ID code for TSL2561
    return 0;
  }
  _tsl2561Initialised = 1;

  /* Set default integration time and gain */
  TSL2561_setIntegrationTime(_tsl2561IntegrationTime);
  TSL2561_setGain(_tsl2561Gain);

  /* Note: by default, the device is in power down mode on bootup */
  if(_gotoSleep)TSL2561_Disable();

  return 1;
}

/**************************************************************************/
/*!
    @brief  Enables or disables the auto-gain settings when reading
            data from the sensor
    @param enable Set to true to enable, False to disable
*/
/**************************************************************************/
void TSL2561_enableAutoRange(_Bool enable)
{
   _tsl2561AutoGain = enable;
}

/**************************************************************************/
/*!
    @brief      Sets the integration time for the TSL2561. Higher time means
                more light captured (better for low light conditions) but will
		take longer to run readings.
    @param time The amount of time we'd like to add up values
*/
/**************************************************************************/
void TSL2561_setIntegrationTime(tsl2561IntegrationTime_t time)
{
  if (!_tsl2561Initialised) TSL2561_Init();

  /* Enable the device by setting the control bit to 0x03 */
  TSL2561_Enable();

  /* Update the timing register */
  Reg_Buf[0] = time | _tsl2561Gain;
  i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,Reg_Buf,1);



  /* Update value placeholders */
  _tsl2561IntegrationTime = time;

  /* Turn the device off to save power */
  if(_gotoSleep)TSL2561_Disable();
}

/**************************************************************************/
/*!
    @brief  Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
    @param gain The value we'd like to set the gain to
*/
/**************************************************************************/
void TSL2561_setGain(tsl2561Gain_t gain)
{
  if (!_tsl2561Initialised)  TSL2561_Init();

  /* Enable the device by setting the control bit to 0x03 */
  TSL2561_Enable();

  /* Update the timing register */
  Reg_Buf[0] = _tsl2561IntegrationTime | gain;
    i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,Reg_Buf,1);


  /* Update value placeholders */
  _tsl2561Gain = gain;

  /* Turn the device off to save power */
  if(_gotoSleep)TSL2561_Disable();
}

/**************************************************************************/
/*!
    @brief  Gets the broadband (mixed lighting) and IR only values from
            the TSL2561, adjusting gain if auto-gain is enabled
    @param  broadband Pointer to a uint16_t we will fill with a sensor
                      reading from the IR+visible light diode.
    @param  ir Pointer to a uint16_t we will fill with a sensor the
               IR-only light diode.
*/
/**************************************************************************/
void TSL2561_getLuminosity (uint16_t *broadband, uint16_t *ir)
{
  _Bool valid = 0;

  if (!_tsl2561Initialised) TSL2561_Init();

  /* If Auto gain disabled get a single reading and continue */
  if(!_tsl2561AutoGain)
  {
	  TSL2561_getData (broadband, ir);
    return;
  }

  /* Read data until we find a valid range */
  _Bool _agcCheck = 0;
  do
  {
    uint16_t _b, _ir;
    uint16_t _hi, _lo;
    tsl2561IntegrationTime_t _it = _tsl2561IntegrationTime;

    /* Get the hi/low threshold for the current integration time */
    switch(_it)
    {
      case TSL2561_INTEGRATIONTIME_13MS:
        _hi = TSL2561_AGC_THI_13MS;
        _lo = TSL2561_AGC_TLO_13MS;
        break;
      case TSL2561_INTEGRATIONTIME_101MS:
        _hi = TSL2561_AGC_THI_101MS;
        _lo = TSL2561_AGC_TLO_101MS;
        break;
      default:
        _hi = TSL2561_AGC_THI_402MS;
        _lo = TSL2561_AGC_TLO_402MS;
        break;
    }

    TSL2561_getData(&_b, &_ir);

    /* Run an auto-gain check if we haven't already done so ... */
    if (!_agcCheck)
    {
      if ((_b < _lo) && (_tsl2561Gain == TSL2561_GAIN_1X))
      {
        /* Increase the gain and try again */
    	  TSL2561_setGain(TSL2561_GAIN_16X);
        /* Drop the previous conversion results */
    	  TSL2561_getData(&_b, &_ir);
        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = 1;
      }
      else if ((_b > _hi) && (_tsl2561Gain == TSL2561_GAIN_16X))
      {
        /* Drop gain to 1x and try again */
    	  TSL2561_setGain(TSL2561_GAIN_1X);
        /* Drop the previous conversion results */
        TSL2561_getData(&_b, &_ir);
        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = 1;
      }
      else
      {
        /* Nothing to look at here, keep moving ....
           Reading is either valid, or we're already at the chips limits */
        *broadband = _b;
        *ir = _ir;
        valid = 1;
      }
    }
    else
    {
      /* If we've already adjusted the gain once, just return the new results.
         This avoids endless loops where a value is at one extreme pre-gain,
         and the the other extreme post-gain */
      *broadband = _b;
      *ir = _ir;
      valid = 1;
    }
  } while (!valid);
}



/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void TSL2561_Enable(void)
{
	/* Enable the device by setting the control bit to 0x03 */
	Reg_Buf[0] =  TSL2561_CONTROL_POWERON;
	i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,Reg_Buf,1);


}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void TSL2561_Disable(void)
{
	/* Turn the device off to save power */
	Reg_Buf[0] =  TSL2561_CONTROL_POWEROFF;
	i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,Reg_Buf,1);


}

/**************************************************************************/
/*!
    function to read luminosity on both channels
*/
/**************************************************************************/
void TSL2561_getData (uint16_t *broadband, uint16_t *ir)
{
  /* Enable the device by setting the control bit to 0x03 */
	TSL2561_Enable();

  /* Wait x ms for ADC to complete */
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      HAL_Delay(TSL2561_DELAY_INTTIME_13MS);  // KTOWN: Was 14ms
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      HAL_Delay(TSL2561_DELAY_INTTIME_101MS); // KTOWN: Was 102ms
      break;
    default:
      HAL_Delay(TSL2561_DELAY_INTTIME_402MS); // KTOWN: Was 403ms
      break;
  }

  /* Reads a two byte value from channel 0 (visible + infrared) */


  i2c_reg_read(_addr,TSL2561_COMMAND_BIT |TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW,Reg_Buf,2);

  *broadband = (Reg_Buf[1]<<8) | Reg_Buf[0];



  /* Reads a two byte value from channel 1 (infrared) */

  i2c_reg_read(_addr,TSL2561_COMMAND_BIT |TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW,Reg_Buf,2);

  *ir = (Reg_Buf[1]<<8) | Reg_Buf[0];




  /* Turn the device off to save power */
  if(_gotoSleep)TSL2561_Disable();
}


/**************************************************************************/
/*!
    @brief  Converts the raw sensor values to the standard SI lux equivalent.
    @param  broadband The 16-bit sensor reading from the IR+visible light diode.
    @param  ir The 16-bit sensor reading from the IR-only light diode.
    @returns The integer Lux value we calcuated.
             Returns 0 if the sensor is saturated and the values are
             unreliable, or 65536 if the sensor is saturated.
*/
/**************************************************************************/
uint32_t TSL2561_calculateLux(uint16_t broadband, uint16_t ir)
{
  unsigned long chScale;
  unsigned long channel1;
  unsigned long channel0;

  /* Make sure the sensor isn't saturated! */
  uint16_t clipThreshold;
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      clipThreshold = TSL2561_CLIPPING_13MS;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      clipThreshold = TSL2561_CLIPPING_101MS;
      break;
    default:
      clipThreshold = TSL2561_CLIPPING_402MS;
      break;
  }

  /* Return 65536 lux if the sensor is saturated */
  if ((broadband > clipThreshold) || (ir > clipThreshold))
  {
    return 65536;
  }

  /* Get the correct scale depending on the intergration time */
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
      break;
    default: /* No scaling ... integration time = 402ms */
      chScale = (1 << TSL2561_LUX_CHSCALE);
      break;
  }

  /* Scale for gain (1x or 16x) */
  if (!_tsl2561Gain) chScale = chScale << 4;

  /* Scale the channel values */
  channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;

  /* Find the ratio of the channel values (Channel1/Channel0) */
  unsigned long ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  /* round the ratio value */
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
  else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
  else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
  else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
  else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
  else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
  else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
  else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

  unsigned long temp;
  channel0 = channel0 * b;
  channel1 = channel1 * m;

  temp = 0;
  /* Do not allow negative lux value */
  if (channel0 > channel1) temp = channel0 - channel1;

  /* Round lsb (2^(LUX_SCALE-1)) */
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  /* Strip off fractional portion */
  uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

  /* Signal I2C had no errors */
  return lux;
}

/**************************************************************************/
/*!
    @brief  Converts the raw sensor values to the standard SI lux equivalent.
    @param  Data The 8-bit value which needs to be written in INTERRUPT register
    @param  High The 16-bit High threshold value for interrupt boundaries
    @param  Low The 16-bit Low threshold value for interrupt boundaries

*/
/**************************************************************************/
void TSL2561_Config_Intr(uint8_t Data,uint16_t *High,uint16_t* Low)
{

	i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_REGISTER_INTERRUPT,&Data,1);


	Reg_Buf[0] =(uint8_t)(*Low & 0xff);
	Reg_Buf[1] = (uint8_t)(*Low >>8);
	i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_WORD_BIT  | TSL2561_REGISTER_THRESHHOLDL_LOW,Reg_Buf,2);

	Reg_Buf[0] =(uint8_t)(*High & 0xff);
	Reg_Buf[1] = (uint8_t)(*High >>8);
	i2c_reg_write(_addr,TSL2561_COMMAND_BIT | TSL2561_WORD_BIT  |TSL2561_REGISTER_THRESHHOLDH_LOW,Reg_Buf,2);

	i2c_reg_read(_addr,TSL2561_COMMAND_BIT | TSL2561_WORD_BIT  |TSL2561_REGISTER_THRESHHOLDL_LOW,Reg_Buf,2);
	*Low = (Reg_Buf[1]<<8) | Reg_Buf[0];   //Updated value of Low , to check if it is successfully written or not


	i2c_reg_read(_addr,TSL2561_COMMAND_BIT | TSL2561_WORD_BIT |TSL2561_REGISTER_THRESHHOLDH_LOW,Reg_Buf,2);
	*High = (Reg_Buf[1]<<8) | Reg_Buf[0]; //Updated value of High , to check if it is successfully written or not


	if(_gotoSleep)TSL2561_Disable();
	return;
}

/**************************************************************************/
/*!
    @brief  Clears the interrupt(Used when sensor in level interrupt mode).
*/
/**************************************************************************/
void TSL2561_Clr_Int()
{

	i2c_only_reg_write(_addr,TSL2561_COMMAND_BIT|TSL2561_CLEAR_BIT);

	if(_gotoSleep)TSL2561_Disable();
    return;

}
