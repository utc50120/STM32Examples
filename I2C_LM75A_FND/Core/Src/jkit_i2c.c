/*
 * jkit_i2c.c
 *
 *  Created on: Aug 19, 2024
 *      Author: jin_mocom
 */
#include "main.h"
#include "jkit_i2c.h"

#define	I2C_WRITE						0
#define	I2C_READ						1
#define	LM75A_ADDR						0x4c
#define	LM75A_CONFIGURATION_REG_ADDR	0x01
#define	LM75A_TEMPERATURE_REG_ADDR		0x00

uint32_t	_jkit_i2c_curr_timer = 0;
uint32_t	_jkit_i2c_update_timer = 100;
int			_jkit_i2c_temperature = 0;
int			_jkit_i2c_temp_value = 0;
char		_jkit_i2c_sign_minus = 0;
uint8_t		_jkit_i2c_buffer[2];

void JKIT_I2C_Initialize(void)
{
	*_jkit_i2c_buffer = 0;
	HAL_I2C_Mem_Write(&hi2c1, ((LM75A_ADDR << 1) | I2C_WRITE), LM75A_CONFIGURATION_REG_ADDR, 1, _jkit_i2c_buffer, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, ((LM75A_ADDR << 1) | I2C_WRITE), LM75A_TEMPERATURE_REG_ADDR, 1, _jkit_i2c_buffer, 2, 100);
}

int JKIT_I2C_Process(void)
{
	_jkit_i2c_curr_timer = HAL_GetTick();

	if(_jkit_i2c_update_timer < _jkit_i2c_curr_timer)
	{
		_jkit_i2c_update_timer = _jkit_i2c_curr_timer + 100;

	  if(HAL_I2C_Master_Receive(&hi2c1, ((LM75A_ADDR << 1) | I2C_WRITE), _jkit_i2c_buffer, 2, 100) == HAL_OK)
	  {
		  _jkit_i2c_temperature = (_jkit_i2c_buffer[0] << 8) | _jkit_i2c_buffer[1];
		  if(_jkit_i2c_temperature & (1<<15))
		  {
			  _jkit_i2c_sign_minus = 1;
			  _jkit_i2c_temperature = (~_jkit_i2c_temperature & 0x7fe) + 0x20;
		  }
		  else
		  {
			  _jkit_i2c_sign_minus = 0;
		  }

		  _jkit_i2c_temperature >>= 5;
		  _jkit_i2c_temperature = (int)(_jkit_i2c_temperature * 1.25);

		  if(_jkit_i2c_sign_minus)
		  {
			  if(_jkit_i2c_temperature > 550)
				  _jkit_i2c_temperature = 550;
		  }
		  else
		  {
			  if(_jkit_i2c_temperature > 1250)
				  _jkit_i2c_temperature = 1250;
		  }

		  _jkit_i2c_temp_value = _jkit_i2c_sign_minus ? -_jkit_i2c_temperature : _jkit_i2c_temperature;
	  }
	  else
	  {
		  return 1;
	  }
	}

	return 0;
}

int JKIT_I2C_GetValue()
{
	return _jkit_i2c_temp_value;
}
