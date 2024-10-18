/*
 * jkit_i2c.h
 *
 *  Created on: Aug 19, 2024
 *      Author: jin_mocom
 */

#ifndef JKIT_I2C_H_
#define JKIT_I2C_H_

extern I2C_HandleTypeDef hi2c1;

/* return 0: no error, other: error exist */
int JKIT_I2C_Process(void);
void JKIT_I2C_Initialize(void);
int JKIT_I2C_GetValue();

#endif /* JKIT_I2C_H_ */
