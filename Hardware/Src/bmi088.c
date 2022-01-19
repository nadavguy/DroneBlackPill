/*
 * bmi088.c
 *
 *  Created on: Jan 17, 2022
 *      Author: raing
 */


/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/

#include "bmi088.h"
#include "bmi08x.h"
#include "BMI088Common.h"
#include "IMUAgent.h"

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

///*!
// *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
// *  range 2G, 4G, 8G or 16G.
// *
// *  @param[in] val       : LSB from each axis.
// *  @param[in] g_range   : Gravity range.
// *  @param[in] bit_width : Resolution for accel.
// *
// *  @return Gravity.
// */
//static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);
//
///*!
// *  @brief This function converts lsb to degree per second for 16 bit gyro at
// *  range 125, 250, 500, 1000 or 2000dps.
// *
// *  @param[in] val       : LSB from each axis.
// *  @param[in] dps       : Degree per second.
// *  @param[in] bit_width : Resolution for gyro.
// *
// *  @return Degree per second.
// */
//static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
//
///*!
// * @brief    This internal API is used to initialize the bmi08x sensor with default
// */
//static int8_t init_bmi08x(struct bmi08x_dev *dev);

int8_t init_bmi08x(struct bmi08x_dev *dev)
{
	int8_t rslt;

	rslt = bmi08a_init(dev);
	bmi08x_error_codes_print_result("bmi08a_init", rslt);

	if (rslt == BMI08X_OK)
	{
		rslt = bmi08g_init(dev);
		bmi08x_error_codes_print_result("bmi08g_init", rslt);
	}

	if (rslt == BMI08X_OK)
	{
		printf("Uploading config file !\n");
		rslt = bmi08a_load_config_file(dev);
		bmi08x_error_codes_print_result("bmi08a_load_config_file", rslt);
	}

	if (rslt == BMI08X_OK)
	{
		dev->accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;

		if (dev->variant == BMI085_VARIANT)
		{
			dev->accel_cfg.range = BMI085_ACCEL_RANGE_16G;
		}
		else if (dev->variant == BMI088_VARIANT)
		{
			dev->accel_cfg.range = BMI088_ACCEL_RANGE_24G;
		}

		dev->accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
		dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

		rslt = bmi08a_set_power_mode(dev);
		bmi08x_error_codes_print_result("bmi08a_set_power_mode", rslt);

		rslt = bmi08a_set_meas_conf(dev);
		bmi08x_error_codes_print_result("bmi08a_set_meas_conf", rslt);

		dev->gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
		dev->gyro_cfg.range = BMI08X_GYRO_RANGE_250_DPS;
		dev->gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
		dev->gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

		rslt = bmi08g_set_power_mode(dev);
		bmi08x_error_codes_print_result("bmi08g_set_power_mode", rslt);

		rslt = bmi08g_set_meas_conf(dev);
		bmi08x_error_codes_print_result("bmi08g_set_meas_conf", rslt);
	}

	return rslt;

}

float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    float half_scale = ((1 << bit_width) / 2.0f);

    gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI08X_GYRO_RANGE_2000_DPS)) * (val);
}

/*!
 *  @brief This API is used to enable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
int8_t enable_bmi08x_interrupt()
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08X_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);

        rslt = bmi08g_get_regs(BMI08X_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi);
        bmi08x_error_codes_print_result("bmi08g_get_regs", rslt);
    }

    return rslt;
}

/*!
 *  @brief This API is used to disable bmi08x interrupt
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
int8_t disable_bmi08x_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08X_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08X_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08x_accel_int_channel_cfg*)&accel_int_config, &bmi);
    bmi08x_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08X_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08X_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08X_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

        /* Disable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08x_gyro_int_channel_cfg *)&gyro_int_config, &bmi);
        bmi08x_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}
