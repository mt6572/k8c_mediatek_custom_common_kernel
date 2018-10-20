/* BOSCH Accelerometer Sensor Driver Header File
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef BMA2XX_H
#define BMA2XX_H

#include <linux/ioctl.h>
/*************************************************************
| sensor | chip id | bit number |     7-bit i2c address      |
-------------------------------------------------------------|
| bma220 |  0xDD   |     6      |0x0B(CSB:low),0x0A(CSB:high)|
-------------------------------------------------------------|
| bma222 |  0x03   |     8      |0x08(SDO:low),0x09(SDO:high)|
-------------------------------------------------------------|
| bma222e|  0xF8   |     8      |0x18(SDO:low),0x19(SDO:high)|
-------------------------------------------------------------|
| bma250 |  0x03   |     10     |0x18(SDO:low),0x19(SDO:high)|
-------------------------------------------------------------|
| bma250e|  0xF9   |     10     |0x18(SDO:low),0x19(SDO:high)|
-------------------------------------------------------------|
| bma255 |  0xFA   |     12     |0x18(SDO:low),0x19(SDO:high)|
-------------------------------------------------------------|
| bma280 |  0xFB   |     14     |0x18(SDO:low),0x19(SDO:high)|
*************************************************************/
/*
 * configuration
*/
#define BMA_DRIVER_VERSION "V1.2"
#define BMA_I2C_ADDRESS(SENSOR_NAME, PIN_STATUS) \
	SENSOR_NAME##_I2C_ADDRESS_##PIN_STATUS

/* apply low pass filter on output */
/* #define CONFIG_BMA_LOWPASS */
#define SW_CALIBRATION

#define BMA_AXIS_X				0
#define BMA_AXIS_Y				1
#define BMA_AXIS_Z				2
#define BMA_AXES_NUM				3
#define BMA_DATA_LEN				6
#define BMA_DEV_NAME				"bma2xx"

#define C_MAX_FIR_LENGTH			(32)
#define MAX_SENSOR_NAME				(32)

/* common definition */
#define BMA_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMA_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define BMA_CHIP_ID_REG				0x00
#define BMA_BUFSIZE				128
#define BM160_CHIP_ID                   0XD1


#define BMI160_ACCEL_RANGE_2G           0X03
#define BMI160_ACCEL_RANGE_4G           0X05
#define BMI160_ACCEL_RANGE_8G           0X08
#define BMI160_ACCELRANGE_16G           0X0C

#define BMI160_ACCEL_ODR_RESERVED       0x00
#define BMI160_ACCEL_ODR_0_78HZ         0x01
#define BMI160_ACCEL_ODR_1_56HZ         0x02
#define BMI160_ACCEL_ODR_3_12HZ         0x03
#define BMI160_ACCEL_ODR_6_25HZ         0x04
#define BMI160_ACCEL_ODR_12_5HZ         0x05
#define BMI160_ACCEL_ODR_25HZ           0x06
#define BMI160_ACCEL_ODR_50HZ           0x07
#define BMI160_ACCEL_ODR_100HZ          0x08
#define BMI160_ACCEL_ODR_200HZ          0x09
#define BMI160_ACCEL_ODR_400HZ          0x0A
#define BMI160_ACCEL_ODR_800HZ          0x0B
#define BMI160_ACCEL_ODR_1600HZ         0x0C
#define BMI160_ACCEL_ODR_RESERVED0      0x0D
#define BMI160_ACCEL_ODR_RESERVED1      0x0E
#define BMI160_ACCEL_ODR_RESERVED2      0x0F


#define BMI160_USER_CHIP_ID_ADDR				0x00
#define BMI160_USER_REV_ID_ADDR                 0x01
#define BMI160_USER_ERROR_ADDR					0X02
#define BMI160_USER_PMU_STATUS_ADDR				0X03
#define BMI160_USER_DATA_0_ADDR					0X04
#define BMI160_USER_DATA_1_ADDR					0X05
#define BMI160_USER_DATA_2_ADDR					0X06
#define BMI160_USER_DATA_3_ADDR					0X07
#define BMI160_USER_DATA_4_ADDR					0X08
#define BMI160_USER_DATA_5_ADDR					0X09
#define BMI160_USER_DATA_6_ADDR					0X0A
#define BMI160_USER_DATA_7_ADDR					0X0B
#define BMI160_USER_DATA_8_ADDR					0X0C
#define BMI160_USER_DATA_9_ADDR					0X0D
#define BMI160_USER_DATA_10_ADDR				0X0E
#define BMI160_USER_DATA_11_ADDR				0X0F
#define BMI160_USER_DATA_12_ADDR				0X10
#define BMI160_USER_DATA_13_ADDR				0X11
#define BMI160_USER_DATA_14_ADDR				0X12
#define BMI160_USER_DATA_15_ADDR				0X13
#define BMI160_USER_DATA_16_ADDR				0X14
#define BMI160_USER_DATA_17_ADDR				0X15
#define BMI160_USER_DATA_18_ADDR				0X16
#define BMI160_USER_DATA_19_ADDR				0X17
#define BMI160_USER_SENSORTIME_0_ADDR			0X18
#define BMI160_USER_SENSORTIME_1_ADDR			0X19
#define BMI160_USER_SENSORTIME_2_ADDR			0X1A
#define BMI160_USER_STATUS_ADDR					0X1B
#define BMI160_USER_INT_STATUS_0_ADDR			0X1C
#define BMI160_USER_INT_STATUS_1_ADDR			0X1D
#define BMI160_USER_INT_STATUS_2_ADDR			0X1E
#define BMI160_USER_INT_STATUS_3_ADDR			0X1F
#define BMI160_USER_TEMPERATURE_0_ADDR			0X20
#define BMI160_USER_TEMPERATURE_1_ADDR			0X21
#define BMI160_USER_FIFO_LENGTH_0_ADDR			0X22
#define BMI160_USER_FIFO_LENGTH_1_ADDR			0X23
#define BMI160_USER_FIFO_DATA_ADDR				0X24
#define BMI160_USER_ACC_CONF_ADDR				0X40
#define BMI160_USER_ACC_RANGE_ADDR              0X41
#define BMI160_USER_GYR_CONF_ADDR               0X42
#define BMI160_USER_GYR_RANGE_ADDR              0X43
#define BMI160_USER_MAG_CONF_ADDR				0X44
#define BMI160_USER_FIFO_DOWNS_ADDR             0X45
#define BMI160_USER_FIFO_CONFIG_0_ADDR          0X46
#define BMI160_USER_FIFO_CONFIG_1_ADDR          0X47
#define BMI160_USER_MAG_IF_0_ADDR				0X4B
#define BMI160_USER_MAG_IF_1_ADDR				0X4C
#define BMI160_USER_MAG_IF_2_ADDR				0X4D
#define BMI160_USER_MAG_IF_3_ADDR				0X4E
#define BMI160_USER_MAG_IF_4_ADDR				0X4F
#define BMI160_USER_INTR_ENABLE_0_ADDR				0X50
#define BMI160_USER_INTR_ENABLE_1_ADDR               0X51
#define BMI160_USER_INTR_ENABLE_2_ADDR               0X52
#define BMI160_USER_INTR_OUT_CTRL_ADDR			0X53
#define BMI160_USER_INTR_LATCH_ADDR				0X54
#define BMI160_USER_INTR_MAP_0_ADDR				0X55
#define BMI160_USER_INTR_MAP_1_ADDR				0X56
#define BMI160_USER_INTR_MAP_2_ADDR				0X57
#define BMI160_USER_INTR_DATA_0_ADDR				0X58
#define BMI160_USER_INTR_DATA_1_ADDR				0X59
#define BMI160_USER_INTR_LOWHIGH_0_ADDR			0X5A
#define BMI160_USER_INTR_LOWHIGH_1_ADDR			0X5B
#define BMI160_USER_INTR_LOWHIGH_2_ADDR			0X5C
#define BMI160_USER_INTR_LOWHIGH_3_ADDR			0X5D
#define BMI160_USER_INTR_LOWHIGH_4_ADDR			0X5E
#define BMI160_USER_INT_MOTION_0_ADDR			0X5F
#define BMI160_USER_INT_MOTION_1_ADDR			0X60
#define BMI160_USER_INT_MOTION_2_ADDR			0X61
#define BMI160_USER_INT_MOTION_3_ADDR			0X62
#define BMI160_USER_INT_TAP_0_ADDR				0X63
#define BMI160_USER_INT_TAP_1_ADDR				0X64
#define BMI160_USER_INT_ORIENT_0_ADDR			0X65
#define BMI160_USER_INT_ORIENT_1_ADDR			0X66
#define BMI160_USER_INT_FLAT_0_ADDR				0X67
#define BMI160_USER_INT_FLAT_1_ADDR				0X68
#define BMI160_USER_FOC_CONF_ADDR				0X69
#define BMI160_USER_CONF_ADDR					0X6A
#define BMI160_USER_IF_CONF_ADDR				0X6B
#define BMI160_USER_PMU_TRIGGER_ADDR			0X6C
#define BMI160_USER_SELF_TEST_ADDR				0X6D
#define BMI160_USER_NV_CONF_ADDR				0x70
#define BMI160_USER_OFFSET_0_ADDR				0X71
#define BMI160_USER_OFFSET_1_ADDR				0X72
#define BMI160_USER_OFFSET_2_ADDR				0X73
#define BMI160_USER_OFFSET_3_ADDR				0X74
#define BMI160_USER_OFFSET_4_ADDR				0X75
#define BMI160_USER_OFFSET_5_ADDR				0X76
#define BMI160_USER_OFFSET_6_ADDR				0X77

#define BMI160_USER_STEP_COUNT_0_ADDR				0X78
#define BMI160_USER_STEP_COUNT_1_ADDR				0X79
#define BMI160_USER_STEP_CONFIG_0_ADDR			0X7A
#define BMI160_USER_STEP_CONFIG_1_ADDR			0X7B
/*------------------------------------------------------------------------
* End of Register Definitions of User
*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------
* Start of Register Definitions of CMD
*------------------------------------------------------------------------*/
 #define BMI160_CMD_COMMANDS_ADDR				0X7E
 #define BMI160_CMD_EXT_MODE_ADDR				0X7F


#define BMG160_RANGE_ADDR			0x43

#define BMG160_RANGE_ADDR_RANGE__POS		0
#define BMG160_RANGE_ADDR_RANGE__LEN		3
#define BMG160_RANGE_ADDR_RANGE__MSK		0x07
#define BMG160_RANGE_ADDR_RANGE__REG		BMG160_RANGE_ADDR

#define BMG160_BW_ADDR				0x42

#define BMG160_BW_ADDR__POS			0
#define BMG160_BW_ADDR__LEN			4
#define BMG160_BW_ADDR__MSK			0x0f
#define BMG160_BW_ADDR__REG			BMG160_BW_ADDR

/* STEP_CNT_0  Description - Reg Addr --> 0x78, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_LSB__POS               0
#define BMI160_USER_STEP_COUNT_LSB__LEN               7
#define BMI160_USER_STEP_COUNT_LSB__MSK               0xFF
#define BMI160_USER_STEP_COUNT_LSB__REG	 BMI160_USER_STEP_COUNT_0_ADDR

/* STEP_CNT_1  Description - Reg Addr --> 0x79, Bit -->  0 to 7 */
#define BMI160_USER_STEP_COUNT_MSB__POS               0
#define BMI160_USER_STEP_COUNT_MSB__LEN               7
#define BMI160_USER_STEP_COUNT_MSB__MSK               0xFF
#define BMI160_USER_STEP_COUNT_MSB__REG	 BMI160_USER_STEP_COUNT_1_ADDR

/* STEP_CONFIG_0  Description - Reg Addr --> 0x7A, Bit -->  0 to 7 */
#define BMI160_USER_STEP_CONFIG_ZERO__POS               0
#define BMI160_USER_STEP_CONFIG_ZERO__LEN               7
#define BMI160_USER_STEP_CONFIG_ZERO__MSK               0xFF
#define BMI160_USER_STEP_CONFIG_ZERO__REG	 BMI160_USER_STEP_CONFIG_0_ADDR

/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 and
4 to 7 */
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__POS               0
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__LEN               3
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__MSK               0x07
#define BMI160_USER_STEP_CONFIG_ONE_CNF1__REG	 BMI160_USER_STEP_CONFIG_1_ADDR

#define BMI160_USER_STEP_CONFIG_ONE_CNF2__POS               4
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__LEN               4
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__MSK               0xF0
#define BMI160_USER_STEP_CONFIG_ONE_CNF2__REG	 BMI160_USER_STEP_CONFIG_1_ADDR

/* STEP_CONFIG_1  Description - Reg Addr --> 0x7B, Bit -->  0 to 2 */
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__POS		3
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__LEN		1
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__MSK		0x08
#define BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG	\
BMI160_USER_STEP_CONFIG_1_ADDR





/* Acc_Range Description - Reg Addr --> 0x41, Bit --> 0...3 */
#define BMI160_USER_ACC_RANGE__POS               0
#define BMI160_USER_ACC_RANGE__LEN               4
#define BMI160_USER_ACC_RANGE__MSK               0x0F
#define BMI160_USER_ACC_RANGE__REG               BMI160_USER_ACC_RANGE_ADDR

/* Acc_Conf Description - Reg Addr --> 0x40, Bit --> 0...3 */
#define BMI160_USER_ACC_CONF_ODR__POS               0
#define BMI160_USER_ACC_CONF_ODR__LEN               4
#define BMI160_USER_ACC_CONF_ODR__MSK               0x0F
#define BMI160_USER_ACC_CONF_ODR__REG		BMI160_USER_ACC_CONF_ADDR


/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG	          \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG	            \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__POS               4
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__MSK               0x10
#define BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG	        \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__POS               5
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__MSK               0x20
#define BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG	       \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__POS               6
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_0 Description - Reg Addr --> 0x50, Bit -->7 */
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__POS               7
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__MSK               0x80
#define BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_0_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->3 */
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__POS               3
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__MSK               0x08
#define BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG	          \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->4 */
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__POS               4
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__MSK               0x10
#define BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG	            \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->5 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__POS               5
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__MSK               0x20
#define BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG	              \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_1 Description - Reg Addr --> 0x51, Bit -->6 */
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__POS               6
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__MSK               0x40
#define BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG	           \
BMI160_USER_INTR_ENABLE_1_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->0 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__POS               0
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__MSK               0x01
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->1 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__POS               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__MSK               0x02
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->2 */
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__POS               2
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__MSK               0x04
#define BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_En_2 Description - Reg Addr --> 0x52, Bit -->3 */
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__POS               3
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__LEN               1
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__MSK               0x08
#define BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG	  \
BMI160_USER_INTR_ENABLE_2_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->0 */
#define BMI160_USER_INTR1_EDGE_CTRL__POS               0
#define BMI160_USER_INTR1_EDGE_CTRL__LEN               1
#define BMI160_USER_INTR1_EDGE_CTRL__MSK               0x01
#define BMI160_USER_INTR1_EDGE_CTRL__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->1 */
#define BMI160_USER_INTR1_LEVEL__POS               1
#define BMI160_USER_INTR1_LEVEL__LEN               1
#define BMI160_USER_INTR1_LEVEL__MSK               0x02
#define BMI160_USER_INTR1_LEVEL__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->2 */
#define BMI160_USER_INTR1_OUTPUT_TYPE__POS               2
#define BMI160_USER_INTR1_OUTPUT_TYPE__LEN               1
#define BMI160_USER_INTR1_OUTPUT_TYPE__MSK               0x04
#define BMI160_USER_INTR1_OUTPUT_TYPE__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->3 */
#define BMI160_USER_INTR1_OUTPUT_ENABLE__POS               3
#define BMI160_USER_INTR1_OUTPUT_ENABLE__LEN               1
#define BMI160_USER_INTR1_OUTPUT_ENABLE__MSK               0x08
#define BMI160_USER_INTR1_OUTPUT_ENABLE__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->4 */
#define BMI160_USER_INTR2_EDGE_CTRL__POS               4
#define BMI160_USER_INTR2_EDGE_CTRL__LEN               1
#define BMI160_USER_INTR2_EDGE_CTRL__MSK               0x10
#define BMI160_USER_INTR2_EDGE_CTRL__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->5 */
#define BMI160_USER_INTR2_LEVEL__POS               5
#define BMI160_USER_INTR2_LEVEL__LEN               1
#define BMI160_USER_INTR2_LEVEL__MSK               0x20
#define BMI160_USER_INTR2_LEVEL__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->6 */
#define BMI160_USER_INTR2_OUTPUT_TYPE__POS               6
#define BMI160_USER_INTR2_OUTPUT_TYPE__LEN               1
#define BMI160_USER_INTR2_OUTPUT_TYPE__MSK               0x40
#define BMI160_USER_INTR2_OUTPUT_TYPE__REG               \
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Out_Ctrl Description - Reg Addr --> 0x53, Bit -->7 */
#define BMI160_USER_INTR2_OUTPUT_EN__POS               7
#define BMI160_USER_INTR2_OUTPUT_EN__LEN               1
#define BMI160_USER_INTR2_OUTPUT_EN__MSK               0x80
#define BMI160_USER_INTR2_OUTPUT_EN__REG		\
BMI160_USER_INTR_OUT_CTRL_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->0...3 */
#define BMI160_USER_INTR_LATCH__POS               0
#define BMI160_USER_INTR_LATCH__LEN               4
#define BMI160_USER_INTR_LATCH__MSK               0x0F
#define BMI160_USER_INTR_LATCH__REG               BMI160_USER_INTR_LATCH_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->4 */
#define BMI160_USER_INTR1_INPUT_ENABLE__POS               4
#define BMI160_USER_INTR1_INPUT_ENABLE__LEN               1
#define BMI160_USER_INTR1_INPUT_ENABLE__MSK               0x10
#define BMI160_USER_INTR1_INPUT_ENABLE__REG               \
BMI160_USER_INTR_LATCH_ADDR

/* Int_Latch Description - Reg Addr --> 0x54, Bit -->5*/
#define BMI160_USER_INTR2_INPUT_ENABLE__POS               5
#define BMI160_USER_INTR2_INPUT_ENABLE__LEN               1
#define BMI160_USER_INTR2_INPUT_ENABLE__MSK               0x20
#define BMI160_USER_INTR2_INPUT_ENABLE__REG              \
BMI160_USER_INTR_LATCH_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->0 */
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__POS               0
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__MSK               0x01
#define BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG	BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->1 */
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__POS               1
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__MSK               0x02
#define BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG	\
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->2 */
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__POS               2
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__MSK               0x04
#define BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->3 */
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__POS               3
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__MSK               0x08
#define BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->4 */
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__POS               4
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__MSK               0x10
#define BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG	\
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->5 */
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__POS               5
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__MSK               0x20
#define BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG	      \
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x55, Bit -->6 */
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__POS               6
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__MSK               0x40
#define BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG	          \
BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_0 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__POS               7
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__LEN               1
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__MSK               0x80
#define BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG	BMI160_USER_INTR_MAP_0_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->0 */
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__POS               0
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__MSK               0x01
#define BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->1 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__POS               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__MSK               0x02
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG	         \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->2 */
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__POS               2
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__MSK               0x04
#define BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG	         \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->3 */
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__POS               3
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__MSK               0x08
#define BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG	      \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->4 */
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__POS               4
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__MSK               0x10
#define BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->5 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__POS               5
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__MSK               0x20
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG	       \
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->6 */
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__POS               6
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__MSK               0x40
#define BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG	\
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_1 Description - Reg Addr --> 0x56, Bit -->7 */
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__POS               7
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__LEN               1
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__MSK               0x80
#define BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG	\
BMI160_USER_INTR_MAP_1_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->0 */
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__POS               0
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__MSK               0x01
#define BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG	BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->1 */
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__POS               1
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__MSK               0x02
#define BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->2 */
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__POS               2
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__MSK               0x04
#define BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->3 */
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__POS               3
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__MSK               0x08
#define BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->4 */
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__POS               4
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__MSK               0x10
#define BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->5 */
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__POS               5
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__MSK               0x20
#define BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->6 */
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__POS               6
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__MSK               0x40
#define BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG	\
BMI160_USER_INTR_MAP_2_ADDR

/* Int_Map_2 Description - Reg Addr --> 0x57, Bit -->7 */

#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__POS               7
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__LEN               1
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__MSK               0x80
#define BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG	BMI160_USER_INTR_MAP_2_ADDR


/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 3 */
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__POS               3
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__LEN               1
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__MSK               0x08
#define BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG	           \
BMI160_USER_INTR_DATA_0_ADDR


/* Int_Data_0 Description - Reg Addr --> 0x58, Bit --> 7 */
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__POS           7
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__LEN           1
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__MSK           0x80
#define BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG            \
BMI160_USER_INTR_DATA_0_ADDR

/*********************************[BMA220]*************************************/
/* chip id */
#define BMA220_CHIP_ID				0xDD

/* i2c address */
#define BMA220_I2C_ADDRESS_PIN_HIGH		0x0A
#define BMA220_I2C_ADDRESS_PIN_LOW		0x0B

/* bandwidth */
#define BMA220_BANDWIDTH_50HZ			0x05
#define BMA220_BANDWIDTH_75HZ			0x04
#define BMA220_BANDWIDTH_150HZ			0x03
#define BMA220_BANDWIDTH_250HZ			0x02
#define BMA220_BANDWIDTH_600HZ			0x01
#define BMA220_BANDWIDTH_1000HZ			0x00

#define BMA220_BANDWIDTH_CONFIG_REG		0x20

#define BMA220_SC_FILT_CONFIG__POS		0
#define BMA220_SC_FILT_CONFIG__MSK		0x0F
#define BMA220_SC_FILT_CONFIG__LEN		4
#define BMA220_SC_FILT_CONFIG__REG		BMA220_BANDWIDTH_CONFIG_REG

/* range */
#define BMA220_RANGE_2G				0
#define BMA220_RANGE_4G				1
#define BMA220_RANGE_8G				2

#define BMA220_RANGE_SELFTEST_REG		0x22

#define BMA220_RANGE__POS			0
#define BMA220_RANGE__MSK			0x03
#define BMA220_RANGE__LEN			2
#define BMA220_RANGE__REG			BMA220_RANGE_SELFTEST_REG

/* power mode */
#define	BMA220_SUSPEND_MODE			0
#define BMA220_NORMAL_MODE			1

#define BMA220_SUSPEND_MODE_REG			0x30

#define BMA220_SUSPEND__POS			0
#define BMA220_SUSPEND__MSK			0xFF
#define BMA220_SUSPEND__LEN			8
#define BMA220_SUSPEND__REG			BMA220_SUSPEND_MODE_REG

/* data */
#define BMA220_REG_DATAXLOW			0x04

/*********************************[BMA222]*************************************/
/* chip id */
#define BMA222_BMA250_CHIP_ID			0x03

/* i2c address */
#define BMA222_I2C_ADDRESS_PIN_HIGH		0x09
#define BMA222_I2C_ADDRESS_PIN_LOW		0x08

/* bandwidth */
#define BMA222_BW_7_81HZ			0x08
#define BMA222_BW_15_63HZ			0x09
#define BMA222_BW_31_25HZ			0x0A
#define BMA222_BW_62_50HZ			0x0B
#define BMA222_BW_125HZ				0x0C
#define BMA222_BW_250HZ				0x0D
#define BMA222_BW_500HZ				0x0E
#define BMA222_BW_1000HZ			0x0F

#define BMA222_BW_SEL_REG			0x10

#define BMA222_BANDWIDTH__POS			0
#define BMA222_BANDWIDTH__LEN			5
#define BMA222_BANDWIDTH__MSK			0x1F
#define BMA222_BANDWIDTH__REG			BMA222_BW_SEL_REG

/* range */
#define BMA222_RANGE_2G				3
#define BMA222_RANGE_4G				5
#define BMA222_RANGE_8G				8

#define BMA222_RANGE_SEL_REG			0x0F

#define BMA222_RANGE_SEL__POS			0
#define BMA222_RANGE_SEL__LEN			4
#define BMA222_RANGE_SEL__MSK			0x0F
#define BMA222_RANGE_SEL__REG			BMA222_RANGE_SEL_REG

/* power mode */
#define BMA222_MODE_SUSPEND			1
#define BMA222_MODE_NORMAL			0

#define BMA222_MODE_CTRL_REG			0x11

#define BMA222_EN_LOW_POWER__POS		6
#define BMA222_EN_LOW_POWER__LEN		1
#define BMA222_EN_LOW_POWER__MSK		0x40
#define BMA222_EN_LOW_POWER__REG		BMA222_MODE_CTRL_REG

#define BMA222_EN_SUSPEND__POS			7
#define BMA222_EN_SUSPEND__LEN			1
#define BMA222_EN_SUSPEND__MSK			0x80
#define BMA222_EN_SUSPEND__REG			BMA222_MODE_CTRL_REG

/* data */
#define BMA222_REG_DATAXLOW			0x02

/*********************************[BMA222E]************************************/
/* chip id */
#define BMA222E_CHIP_ID				0xF8

/* i2c address */
#define BMA2X2_I2C_ADDRESS_PIN_HIGH		0x19
#define BMA2X2_I2C_ADDRESS_PIN_LOW		0x18

#define BMA222E_I2C_ADDRESS_PIN_HIGH		0x19
#define BMA222E_I2C_ADDRESS_PIN_LOW		0x18

/* bandwidth */
#define BMA2X2_BW_7_81HZ			0x08
#define BMA2X2_BW_15_63HZ			0x09
#define BMA2X2_BW_31_25HZ			0x0A
#define BMA2X2_BW_62_50HZ			0x0B
#define BMA2X2_BW_125HZ				0x0C
#define BMA2X2_BW_250HZ				0x0D
#define BMA2X2_BW_500HZ				0x0E
#define BMA2X2_BW_1000HZ			0x0F

#define BMA2X2_BW_SEL_REG			0x10

#define BMA2X2_BANDWIDTH__POS			0
#define BMA2X2_BANDWIDTH__LEN			5
#define BMA2X2_BANDWIDTH__MSK			0x1F
#define BMA2X2_BANDWIDTH__REG			BMA2X2_BW_SEL_REG

/* range */
#define BMA2X2_RANGE_2G				3
#define BMA2X2_RANGE_4G				5
#define BMA2X2_RANGE_8G				8
#define BMA2X2_RANGE_16G			12

#define BMA2X2_RANGE_SEL_REG			0x0F

#define BMA2X2_RANGE_SEL__POS			0
#define BMA2X2_RANGE_SEL__LEN			4
#define BMA2X2_RANGE_SEL__MSK			0x0F
#define BMA2X2_RANGE_SEL__REG			BMA2X2_RANGE_SEL_REG

/* power mode */
#define BMA2X2_MODE_NORMAL			0
#define BMA2X2_MODE_SUSPEND			4

#define BMA2X2_MODE_CTRL_REG			0x11

#define BMA2X2_MODE_CTRL__POS			5
#define BMA2X2_MODE_CTRL__LEN			3
#define BMA2X2_MODE_CTRL__MSK			0xE0
#define BMA2X2_MODE_CTRL__REG			BMA2X2_MODE_CTRL_REG

#define BMA2X2_LOW_NOISE_CTRL_REG		0x12

#define BMA2X2_LOW_POWER_MODE__POS		6
#define BMA2X2_LOW_POWER_MODE__LEN		1
#define BMA2X2_LOW_POWER_MODE__MSK		0x40
#define BMA2X2_LOW_POWER_MODE__REG		BMA2X2_LOW_NOISE_CTRL_REG

/* data */
#define BMA2X2_REG_DATAXLOW			0x02

/*********************************[BMA250]*************************************/
/* chip id */
/* bma250 chip id is same as bma222 */

/* i2c address */
#define BMA250_I2C_ADDRESS_PIN_HIGH		0x19
#define BMA250_I2C_ADDRESS_PIN_LOW		0x18

/* bandwidth */
#define BMA250_BW_7_81HZ			0x08
#define BMA250_BW_15_63HZ			0x09
#define BMA250_BW_31_25HZ			0x0A
#define BMA250_BW_62_50HZ			0x0B
#define BMA250_BW_125HZ				0x0C
#define BMA250_BW_250HZ				0x0D
#define BMA250_BW_500HZ				0x0E
#define BMA250_BW_1000HZ			0x0F

#define BMA250_BW_SEL_REG			0x10

#define BMA250_BANDWIDTH__POS			0
#define BMA250_BANDWIDTH__LEN			5
#define BMA250_BANDWIDTH__MSK			0x1F
#define BMA250_BANDWIDTH__REG			BMA250_BW_SEL_REG

/* range */
#define BMA250_RANGE_2G				3
#define BMA250_RANGE_4G				5
#define BMA250_RANGE_8G				8

#define BMA250_RANGE_SEL_REG			0x0F

#define BMA250_RANGE_SEL__POS			0
#define BMA250_RANGE_SEL__LEN			4
#define BMA250_RANGE_SEL__MSK			0x0F
#define BMA250_RANGE_SEL__REG			BMA250_RANGE_SEL_REG

/* power mode */
#define BMA250_MODE_NORMAL			0
#define BMA250_MODE_SUSPEND			1

#define BMA250_MODE_CTRL_REG			0x11

#define BMA250_EN_LOW_POWER__POS		6
#define BMA250_EN_LOW_POWER__LEN		1
#define BMA250_EN_LOW_POWER__MSK		0x40
#define BMA250_EN_LOW_POWER__REG		BMA250_MODE_CTRL_REG

#define BMA250_EN_SUSPEND__POS			7
#define BMA250_EN_SUSPEND__LEN			1
#define BMA250_EN_SUSPEND__MSK			0x80
#define BMA250_EN_SUSPEND__REG			BMA250_MODE_CTRL_REG

/* data */
#define BMA250_REG_DATAXLOW			0x02

/*********************************[BMA250E]************************************/
/* chip id */
#define BMA250E_CHIP_ID				0xF9

/* i2c address */
#define BMA250E_I2C_ADDRESS_PIN_HIGH		0x19
#define BMA250E_I2C_ADDRESS_PIN_LOW		0x18

/* bandwidth */
/*bma250e bandwidth is same as bma222e */

/* range */
/*bma250e range is same as bma222e */

/* power mode */
/*bma250e powermode is same as bma222e */

/* data */
/*bma250e data register is same as bma222e */

/*********************************[BMA255]*************************************/
/* chip id */
#define BMA255_CHIP_ID				0xFA

/* i2c address */
#define BMA255_I2C_ADDRESS_PIN_HIGH		0x19
#define BMA255_I2C_ADDRESS_PIN_LOW		0x18

/* bandwidth */
/*bma255 bandwidth is same as bma222e */

/* range */
/*bma255 range is same as bma222e */

/* power mode */
/*bma255 powermode is same as bma222e */

/* data */
/*bma255 data register is same as bma222e */

/*********************************[BMA280]*************************************/
/* chip id */
#define BMA280_CHIP_ID				0xFB

/* i2c address */
#define BMA280_I2C_ADDRESS_PIN_HIGH		0x19
#define BMA280_I2C_ADDRESS_PIN_LOW		0x18

/* bandwidth */
/*
*bma280 bandwidth is same as bma222e except 1000Hz,
*bma280 doesn't support 1000Hz bandwidth.
*/

/* range */
/*bma280 range is same as bma222e */

/* power mode */
/*bma280 powermode is same as bma222e */

/* data */
/*bma280 data register is same as bma222e */

#endif/* BMA2XX_H */
