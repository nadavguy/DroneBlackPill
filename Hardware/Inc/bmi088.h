/*
 * bmi088.h
 *
 *  Created on: Jan 17, 2022
 *      Author: raing
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "bmi08x_defs.h"

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

extern float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);
extern float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
extern int8_t init_bmi08x(struct bmi08x_dev *dev);
extern int8_t enable_bmi08x_interrupt(void);
extern int8_t disable_bmi08x_interrupt(void);

#endif /* INC_BMI088_H_ */
