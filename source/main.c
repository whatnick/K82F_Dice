/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/
#include <stdlib.h>

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "seven_seg.h"

#include "math.h"
#include "fsl_fxos.h"
#include "fsl_i2c.h"

#include "fsl_trng.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C3_CLK_SRC
#define I2C_BAUDRATE 100000U

/* Number of TRNG generated */
#define TRNG_EXAMPLE_RANDOM_NUMBER 10

/*******************************************************************************
 * Variables
 ******************************************************************************/
i2c_master_handle_t g_MasterHandle;
/* FXOS device address */
const uint8_t g_accel_address[] = { 0x1CU, 0x1DU, 0x1EU, 0x1FU };

void delay(uint8_t t) {
	volatile uint32_t i = 0;
	volatile uint32_t delay_t = 800000 * t;
	for (i = 0; i < delay_t; ++i) {
		__asm("NOP");
		/* delay */
	}
}

/*!
 * @brief Application entry point.
 */
int main(void) {
	fxos_handle_t fxosHandle = { 0 };
	fxos_data_t sensorData = { 0 };
	i2c_master_config_t i2cConfig = { 0 };
	uint32_t i2cSourceClock = 0;
	int16_t xData = 0;
	int16_t yData = 0;
	int16_t zData = 0;
	uint8_t i = 0;
	uint8_t dice_val = 0;
	uint8_t regResult = 0;
	uint8_t array_addr_size = 0;
	bool foundDevice = false;

	trng_config_t trngConfig;
	status_t status;
	uint32_t data[TRNG_EXAMPLE_RANDOM_NUMBER];

	i2cSourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);
	fxosHandle.base = BOARD_ACCEL_I2C_BASEADDR;
	fxosHandle.i2cHandle = &g_MasterHandle;

	/* Init board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	/*
	 * i2cConfig.baudRate_Bps = 100000U;
	 * i2cConfig.enableHighDrive = false;
	 * i2cConfig.enableStopHold = false;
	 * i2cConfig.glitchFilterWidth = 0U;
	 * i2cConfig.enableMaster = true;
	 */
	I2C_MasterGetDefaultConfig(&i2cConfig);
	I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &i2cConfig, i2cSourceClock);
	I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_MasterHandle,
	NULL, NULL);

	/* Find sensor devices */
	array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
	for (i = 0; i < array_addr_size; i++) {
		fxosHandle.xfer.slaveAddress = g_accel_address[i];
		if (FXOS_ReadReg(&fxosHandle, WHO_AM_I_REG, &regResult, 1)
				== kStatus_Success) {
			foundDevice = true;
			break;
		}
		if ((i == (array_addr_size - 1)) && (!foundDevice)) {
			PRINTF("\r\nAccel sensor not found\r\n");
			while (1) {
			};
		}
	}

	/* Init accelerometer sensor */
	FXOS_Init(&fxosHandle);

	/* Init 7 Segment Display */
	init_7seg();

	/* Initialize TRNG configuration structure to default.*/
	/*
	 * trngConfig.lock = TRNG_USER_CONFIG_DEFAULT_LOCK;
	 * trngConfig.clockMode = kTRNG_ClockModeRingOscillator;
	 * trngConfig.ringOscDiv = TRNG_USER_CONFIG_DEFAULT_OSC_DIV;
	 * trngConfig.sampleMode = kTRNG_SampleModeRaw;
	 * trngConfig.entropyDelay = TRNG_USER_CONFIG_DEFAULT_ENTROPY_DELAY;
	 * trngConfig.sampleSize = TRNG_USER_CONFIG_DEFAULT_SAMPLE_SIZE;
	 * trngConfig.sparseBitLimit = TRNG_USER_CONFIG_DEFAULT_SPARSE_BIT_LIMIT;
	 * trngConfig.retryCount = TRNG_USER_CONFIG_DEFAULT_RETRY_COUNT;
	 * trngConfig.longRunMaxLimit = TRNG_USER_CONFIG_DEFAULT_RUN_MAX_LIMIT;
	 * trngConfig.monobitLimit.maximum = TRNG_USER_CONFIG_DEFAULT_MONOBIT_MAXIMUM;
	 * trngConfig.monobitLimit.minimum = TRNG_USER_CONFIG_DEFAULT_MONOBIT_MINIMUM;
	 * trngConfig.runBit1Limit.maximum = TRNG_USER_CONFIG_DEFAULT_RUNBIT1_MAXIMUM;
	 * trngConfig.runBit1Limit.minimum = TRNG_USER_CONFIG_DEFAULT_RUNBIT1_MINIMUM;
	 * trngConfig.runBit2Limit.maximum = TRNG_USER_CONFIG_DEFAULT_RUNBIT2_MAXIMUM;
	 * trngConfig.runBit2Limit.minimum = TRNG_USER_CONFIG_DEFAULT_RUNBIT2_MINIMUM;
	 * trngConfig.runBit3Limit.maximum = TRNG_USER_CONFIG_DEFAULT_RUNBIT3_MAXIMUM;
	 * trngConfig.runBit3Limit.minimum = TRNG_USER_CONFIG_DEFAULT_RUNBIT3_MINIMUM;
	 * trngConfig.runBit4Limit.maximum = TRNG_USER_CONFIG_DEFAULT_RUNBIT4_MAXIMUM;
	 * trngConfig.runBit4Limit.minimum = TRNG_USER_CONFIG_DEFAULT_RUNBIT4_MINIMUM;
	 * trngConfig.runBit5Limit.maximum = TRNG_USER_CONFIG_DEFAULT_RUNBIT5_MAXIMUM;
	 * trngConfig.runBit5Limit.minimum = TRNG_USER_CONFIG_DEFAULT_RUNBIT5_MINIMUM;
	 * trngConfig.runBit6PlusLimit.maximum = TRNG_USER_CONFIG_DEFAULT_RUNBIT6PLUS_MAXIMUM;
	 * trngConfig.runBit6PlusLimit.minimum = TRNG_USER_CONFIG_DEFAULT_RUNBIT6PLUS_MINIMUM;
	 * trngConfig.pokerLimit.maximum = TRNG_USER_CONFIG_DEFAULT_POKER_MAXIMUM;
	 * trngConfig.pokerLimit.minimum = TRNG_USER_CONFIG_DEFAULT_POKER_MINIMUM;
	 * trngConfig.frequencyCountLimit.maximum = TRNG_USER_CONFIG_DEFAULT_FREQUENCY_MAXIMUM;
	 * trngConfig.frequencyCountLimit.minimum = TRNG_USER_CONFIG_DEFAULT_FREQUENCY_MINIMUM;
	 */
	TRNG_GetDefaultConfig(&trngConfig);
	/* Set sample mode of the TRNG ring oscillator to Von Neumann, for better random data.*/
	trngConfig.sampleMode = kTRNG_SampleModeVonNeumann;

	/* Initialize TRNG */
	status = TRNG_Init(TRNG0, &trngConfig);

	if (kStatus_Success != status) {
		PRINTF("\r\nTRNG not initialized\r\n");
		while (1) {
		};
	}

	PRINTF("Accelerometer + Ring Oscillator Entropy Dice");

	for (;;) { /* Infinite loop to avoid leaving the main function */
		/* Get new accelerometer data. */
		FXOS_ReadSensorData(&fxosHandle, &sensorData);

		/* Get the X and Y data from the sensor data structure.fxos_data */
		xData = (int16_t) ((uint16_t) ((uint16_t) sensorData.accelXMSB << 8)
				| (uint16_t) sensorData.accelXLSB);
		yData = (int16_t) ((uint16_t) ((uint16_t) sensorData.accelYMSB << 8)
				| (uint16_t) sensorData.accelYLSB);
		zData = (int16_t) ((uint16_t) ((uint16_t) sensorData.accelZMSB << 8)
				| (uint16_t) sensorData.accelZLSB);

		/* Print out the raw accelerometer data. */
		PRINTF("x= %d y = %d z = %d\r\n", xData, yData, zData);

		/* Detect horizontal shake and display a value */
		if (abs(xData) > 2000 || abs(yData) > 2000) {
			clear_display();

			//TODO: Add reasonable derivation of RNG for RAW Accel data
			//Refer:http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7109113
			FXOS_ReadSensorData(&fxosHandle, &sensorData);
			dice_val = ((sensorData.accelXLSB & 15)
					^ (sensorData.accelYLSB & 15) ^ (sensorData.accelZLSB & 15))
					% 10;
			delay(1);
			FXOS_ReadSensorData(&fxosHandle, &sensorData);
			dice_val = (dice_val
					+ ((sensorData.accelXLSB & 15) ^ (sensorData.accelYLSB & 15)
							^ (sensorData.accelZLSB & 15)) % 10) / 2;
			display_num(dice_val);
		}

		dice_val = 0;

		PRINTF("Generate %d random numbers: \r\n", TRNG_EXAMPLE_RANDOM_NUMBER);

		/* Get Random data*/
		status = TRNG_GetRandomData(TRNG0, data, sizeof(data));
		if (status == kStatus_Success) {
			/* Print data*/
			for (i = 0; i < TRNG_EXAMPLE_RANDOM_NUMBER; i++) {
				PRINTF("Random[%d] = 0x%X\r\n", i, data[i]);
			}
		} else {
			PRINTF("TRNG failed!\r\n");
		}
	}
}
