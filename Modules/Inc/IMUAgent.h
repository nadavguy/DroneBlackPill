/*
 * BME680Agent.h
 *
 *  Created on: Jan 16, 2022
 *      Author: raing
 */

#ifndef INC_IMUAGENT_H_
#define INC_IMUAGENT_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "bme68x_defs.h"
#include "bmm150_defs.h"
#include "bmi08x_defs.h"

#define BME68X_VALID_DATA  UINT8_C(0xB0)

extern struct bme68x_dev bme;
extern int8_t rslt;
extern struct bme68x_conf conf;
extern struct bme68x_heatr_conf heatr_conf;
extern struct bme68x_data data[3];
extern uint32_t del_period;
extern uint8_t n_fields;

extern struct bmm150_dev bmm;

extern struct bmi08x_dev bmi;
extern struct bmi08x_sensor_data bmi08x_accel;
extern struct bmi08x_sensor_data bmi08x_gyro;
extern struct bmi08x_accel_int_channel_cfg accel_int_config;
extern struct bmi08x_gyro_int_channel_cfg gyro_int_config;

//extern tAHRSDATA rawARHSData;

extern void initBME680(void);
extern void initBMM150(void);
extern void initBMI088(void);

extern int8_t get_data(struct bmm150_dev *dev);
#ifdef __cplusplus
}
#endif
#endif /* INC_IMUAGENT_H_ */
