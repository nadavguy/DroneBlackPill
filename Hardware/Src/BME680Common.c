/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <BME680Common.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "i2c.h"
#include "bme68x.h"
#include "cmsis_os.h"
//#include "coines.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID  0x93

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
//    uint8_t dev_addr = *(uint8_t*)intf_ptr;
//    uint8_t res = HAL_I2C_Master_Receive_DMA(&hi2c1, reg_addr, reg_data, (uint16_t)len);
	uint8_t res = HAL_I2C_Mem_Read(&hi2c1, (0x76)<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, len, 100);

    return res;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
//    uint8_t dev_addr = *(uint8_t*)intf_ptr;

//    uint8_t res = HAL_I2C_Master_Transmit(&hi2c1, reg_addr, (uint8_t *)reg_data, (uint16_t)len, 25);
	uint8_t res = HAL_I2C_Mem_Write(&hi2c1, (0x76)<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, len, 100);//(&hi2c1, reg_addr, (uint8_t *)reg_data, (uint16_t)len, 25);
    return res;
}

/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
	//80M ticks are 1 second and 1M microseconds
	//80 ticks are 1 microseconds
    osDelay(period);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
	int8_t rslt = BME68X_OK;

	if (bme != NULL)
	{
		/* Bus configuration : I2C */
		if (intf == BME68X_I2C_INTF)
		{
			printf("I2C Interface\n");
			dev_addr = BME68X_I2C_ADDR_LOW;
			bme->read = bme68x_i2c_read;
			bme->write = bme68x_i2c_write;
			bme->intf = BME68X_I2C_INTF;
//			coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
		}
		bme->delay_us = bme68x_delay_us;
		bme->intf_ptr = &hi2c1;
		bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
	}
	else
	{
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

void bme68x_coines_deinit(void)
{
//    fflush(stdout);
//
//    coines_set_shuttleboard_vdd_vddio_config(0, 0);
//    coines_delay_msec(1000);
//
//    /* Coines interface reset */
//    coines_soft_reset();
//    coines_delay_msec(1000);
//    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
