/*
 * bmi088g.h
 *
 *  Created on: Jan 18, 2022
 *      Author: raing
 */

#ifndef INC_BMI088G_H_
#define INC_BMI088G_H_

extern int8_t bmi08g_get_data_int_status(uint8_t *int_status, struct bmi08x_dev *dev);

extern int8_t bmi08g_get_data(struct bmi08x_sensor_data *gyro, struct bmi08x_dev *dev);

#endif /* INC_BMI088G_H_ */
