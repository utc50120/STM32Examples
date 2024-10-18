/*
 * jkit_fnd.c
 *
 *  Created on: Aug 19, 2024
 *      Author: jin_mocom
 */
#include	"main.h"

#include	"jkit_fnd.h"


char		_jkit_fnd_error = 0;
int			_jkit_fnd_number = 0;
uint32_t	_jkit_fnd_timer = 4;
uint32_t	_jkit_fnd_timer2 = 0;
char		_jkit_fnd_pos = 0;
char		_jkit_fnd_minus = 0;
char		_jkit_fnd_digit[4];


void JKIT_FND_display(char position, char number, char dot_display);


void JKIT_FND_process(void)
{
	_jkit_fnd_timer2 = HAL_GetTick();

	if(_jkit_fnd_timer < _jkit_fnd_timer2)
	{
	  _jkit_fnd_timer = _jkit_fnd_timer2;

	  switch(_jkit_fnd_pos)
	  {
	  case 0:
		  JKIT_FND_display(0, _jkit_fnd_digit[0], 0);
		  break;

	  case 1:
		  JKIT_FND_display(1, _jkit_fnd_digit[1], _jkit_fnd_error ? 0 : 1);
		  break;

	  case 2:
		  if(_jkit_fnd_digit[3] == 0 && _jkit_fnd_digit[2] == 0)
			  JKIT_FND_display(2, -1, 0);
		  else
			  JKIT_FND_display(2, _jkit_fnd_digit[2], 0);
		  break;

	  case 3:
		  if(_jkit_fnd_minus && !_jkit_fnd_error)
			  JKIT_FND_display(3, '-', 0);
		  else if(_jkit_fnd_digit[3] == 0)
			  JKIT_FND_display(3, -1, 0);
		  else
			  JKIT_FND_display(3, _jkit_fnd_digit[3], 0);
		  break;
	  }

	  if(++_jkit_fnd_pos > 3)
		  _jkit_fnd_pos = 0;
	}
}

/* number = -999 ~ 9999, display = -99.9 ~ 999.9 */
void JKIT_FND_number(int number, char sign_minus)
{
	int temp1 = 0, temp2 = 0;

	if(_jkit_fnd_error)
		return;

	_jkit_fnd_number = number;
	_jkit_fnd_minus = sign_minus;

	temp2 = number;
	temp1 = temp2 / 1000;
	temp2 -= (temp1 * 1000);
	_jkit_fnd_digit[3] = (char)(temp1);

	temp1 = temp2 / 100;
	temp2 -= (temp1 * 100);
	_jkit_fnd_digit[2] = (char)(temp1);

	temp1 = temp2 / 10;
	temp2 -= (temp1 * 10);
	_jkit_fnd_digit[1] = (char)(temp1);
	_jkit_fnd_digit[0] = (char)(temp2);
}

/* error_number = 0: no error, 1 ~ 9 */
void JKIT_FND_error(char error_number)
{
	_jkit_fnd_error = error_number;
	_jkit_fnd_digit[3] = 'E';
	_jkit_fnd_digit[2] = 'r';
	_jkit_fnd_digit[1] = 'r';
	_jkit_fnd_digit[0] = error_number;
}

/* FND display function
 * position: 0 ~ 3 FND digit select
 * number: 0 ~ 9, -, E, r
 * dot_display: 0, 1
 */
void JKIT_FND_display(char position, char number, char dot_display)
{
	/* clear FND digit select pin */
	HAL_GPIO_WritePin(FND_SEL0_GPIO_Port, FND_SEL0_Pin, 0);
	HAL_GPIO_WritePin(FND_SEL1_GPIO_Port, FND_SEL1_Pin, 0);
	HAL_GPIO_WritePin(FND_SEL2_GPIO_Port, FND_SEL2_Pin, 0);
	HAL_GPIO_WritePin(FND_SEL3_GPIO_Port, FND_SEL3_Pin, 0);

	/* set FND digit select pin */
	switch(position)
	{
	case 0: HAL_GPIO_WritePin(FND_SEL0_GPIO_Port, FND_SEL0_Pin, 1); break;
	case 1:	HAL_GPIO_WritePin(FND_SEL1_GPIO_Port, FND_SEL1_Pin, 1); break;
	case 2:	HAL_GPIO_WritePin(FND_SEL2_GPIO_Port, FND_SEL2_Pin, 1); break;
	case 3:	HAL_GPIO_WritePin(FND_SEL3_GPIO_Port, FND_SEL3_Pin, 1); break;
	}

	/* clear FND character */
	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin|FNDC_Pin|FNDD_Pin|FNDE_Pin|FNDF_Pin|FNDG_Pin, 0);

	/* set FND character */
	switch(number)
	{
	case 0:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin|FNDC_Pin|FNDD_Pin|FNDE_Pin|FNDF_Pin         , 1); break;
	case 1:	HAL_GPIO_WritePin(FNDA_GPIO_Port,          FNDB_Pin|FNDC_Pin                                    , 1); break;
	case 2:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin         |FNDD_Pin|FNDE_Pin         |FNDG_Pin, 1); break;
	case 3:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin|FNDC_Pin|FNDD_Pin                  |FNDG_Pin, 1); break;
	case 4:	HAL_GPIO_WritePin(FNDA_GPIO_Port,          FNDB_Pin|FNDC_Pin                  |FNDF_Pin|FNDG_Pin, 1); break;
	case 5:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin         |FNDC_Pin|FNDD_Pin         |FNDF_Pin|FNDG_Pin, 1); break;
	case 6:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin         |FNDC_Pin|FNDD_Pin|FNDE_Pin|FNDF_Pin|FNDG_Pin, 1); break;
	case 7:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin|FNDC_Pin                  |FNDF_Pin         , 1); break;
	case 8:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin|FNDC_Pin|FNDD_Pin|FNDE_Pin|FNDF_Pin|FNDG_Pin, 1); break;
	case 9:	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin|FNDB_Pin|FNDC_Pin|FNDD_Pin         |FNDF_Pin|FNDG_Pin, 1); break;
	case '-': HAL_GPIO_WritePin(FNDA_GPIO_Port,                                                     FNDG_Pin, 1); break;
	case 'E': HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDA_Pin                |FNDD_Pin|FNDE_Pin|FNDF_Pin|FNDG_Pin, 1); break;
	case 'r': HAL_GPIO_WritePin(FNDA_GPIO_Port,                                   FNDE_Pin         |FNDG_Pin, 1); break;
	}

	/* on/off dot */
	HAL_GPIO_WritePin(FNDA_GPIO_Port, FNDP_Pin, dot_display);
}