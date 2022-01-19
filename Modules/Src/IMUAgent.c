/*
 * BME680Agent.c
 *
 *  Created on: Jan 16, 2022
 *      Author: raing
 */


#include "bme68x.h"
#include "bme68x_defs.h"
#include "BME680Common.h"

#include "bmm150.h"
#include "bmm150_defs.h"
#include "BMM150Common.h"

#include "bmi088.h"
#include "bmi088a.h"
#include "bmi088g.h"
#include "bmi08x.h"
#include "bmi08x_defs.h"
#include "BMI088Common.h"
#include "AHRS.h"


//BME680
struct bme68x_dev bme;
int8_t rslt;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data[3];
uint32_t del_period;
uint8_t n_fields;

/* Heater temperature in degree Celsius */
uint16_t temp_prof[10] = { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 };
/* Multiplier to the shared heater duration */
uint16_t mul_prof[10] = { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 };

//BMM150
struct bmm150_dev bmm;

//BMI088
struct bmi08x_dev bmi;
struct bmi08x_sensor_data bmi08x_accel;
struct bmi08x_sensor_data bmi08x_gyro;
struct bmi08x_accel_int_channel_cfg accel_int_config;
struct bmi08x_gyro_int_channel_cfg gyro_int_config;



static int8_t set_config(struct bmm150_dev *dev);
//static int8_t get_data(struct bmm150_dev *dev);

void initBME680(void)
{
	/* Interface preference is updated as a parameter
	 * For I2C : BME68X_I2C_INTF
	 * For SPI : BME68X_SPI_INTF
	 */
	rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
//	bme68x_check_rslt("bme68x_interface_init", rslt);

	rslt = bme68x_init(&bme);
//	bme68x_check_rslt("bme68x_init", rslt);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	rslt = bme68x_get_conf(&conf, &bme);
//	bme68x_check_rslt("bme68x_get_conf", rslt);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	conf.filter = BME68X_FILTER_OFF;
	conf.odr = BME68X_ODR_NONE;
	conf.os_hum = BME68X_OS_1X;
	conf.os_pres = BME68X_OS_16X;
	conf.os_temp = BME68X_OS_2X;
	rslt = bme68x_set_conf(&conf, &bme);
//	bme68x_check_rslt("bme68x_set_conf", rslt);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp_prof = temp_prof;
	heatr_conf.heatr_dur_prof = mul_prof;

	/* Shared heating duration in milliseconds */
	heatr_conf.shared_heatr_dur = 140 - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000);

	heatr_conf.profile_len = 10;
	rslt = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme);
//	bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	rslt = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme);
//	bme68x_check_rslt("bme68x_set_op_mode", rslt);

	del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme)/1000 + (heatr_conf.shared_heatr_dur);
	bme.delay_us(del_period, bme.intf_ptr);
}

void initBMM150(void)
{
	rslt = bmm150_interface_selection(&bmm);
//	bmm150_error_codes_print_result("bmm150_interface_selection", rslt);

	if (rslt == BMM150_OK)
	{
		rslt = bmm150_init(&bmm);
//		bmm150_error_codes_print_result("bmm150_init", rslt);

		if (rslt == BMM150_OK)
		{
			rslt = set_config(&bmm);
//			bmm150_error_codes_print_result("set_config", rslt);
		}
	}
}

/*!
 *  @brief This internal API is used to set configurations like powermode, odr and interrupt mapping.
 */
static int8_t set_config(struct bmm150_dev *dev)
{
	/* Status of api are returned to this variable. */
	int8_t rslt;

	struct bmm150_settings settings;

	/* Set powermode as normal mode */
	settings.pwr_mode = BMM150_POWERMODE_NORMAL;
	rslt = bmm150_set_op_mode(&settings, dev);
//	bmm150_error_codes_print_result("bmm150_set_op_mode", rslt);

	if (rslt == BMM150_OK)
	{
		/* Setting the preset mode as Low power mode
		 * i.e. data rate = 10Hz, XY-rep = 1, Z-rep = 2
		 */
		settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
		rslt = bmm150_set_presetmode(&settings, dev);
//		bmm150_error_codes_print_result("bmm150_set_presetmode", rslt);

		if (rslt == BMM150_OK)
		{
			/* Map the data interrupt pin */
			settings.int_settings.drdy_pin_en = 0x01;
			rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
//			bmm150_error_codes_print_result("bmm150_set_sensor_settings", rslt);
		}
	}

	return rslt;
}

/*!
 *  @brief This internal API is used to get gyro data.
 */
int8_t get_data(struct bmm150_dev *dev)
{
	/* Status of api are returned to this variable. */
	int8_t rslt;

	struct bmm150_mag_data mag_data;

	/* Get the interrupt status */
	rslt = bmm150_get_interrupt_status(dev);

	if (dev->int_status & BMM150_INT_ASSERTED_DRDY)
	{
		/* Read mag data */
		rslt = bmm150_read_mag_data(&mag_data, dev);
		rawARHSData.MagData[0] = (float)mag_data.x;
		rawARHSData.MagData[1] = (float)mag_data.y;
		rawARHSData.MagData[2] = (float)mag_data.z;
	}

	return rslt;
}

void initBMI088(void)
{
	int8_t rslt;


	/* Interface given as parameter
	 *           For I2C : BMI08X_I2C_INTF
	 *           For SPI : BMI08X_SPI_INTF
	 * Sensor variant given as parameter
	 *          For BMI085 : BMI085_VARIANT
	 *          For BMI088 : BMI088_VARIANT
	 */
	rslt = bmi08x_interface_init(&bmi, BMI08X_I2C_INTF, BMI088_VARIANT);
//	bmi08x_error_codes_print_result("bmi08x_interface_init", rslt);

	if (rslt == BMI08X_OK)
	{
		rslt = init_bmi08x(&bmi);
//		bmi08x_error_codes_print_result("init_bmi08x", rslt);

		/* Enable data ready interrupts */

//		bmi08x_error_codes_print_result("enable_bmi08x_interrupt", rslt);



//			times_to_read = 0;
//
//			printf("\n\nGYRO DATA\n");
//			printf("Gyro data in LSB units and degrees per second\n");
//			printf("Gyro data range : 250 dps for BMI085 and BMI088\n\n");




		/* Disable data ready interrupts */

//		bmi08x_error_codes_print_result("disable_bmi08x_interrupt", rslt);
	}
}
