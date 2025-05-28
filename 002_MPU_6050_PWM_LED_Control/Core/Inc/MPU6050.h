/*
 * MPU6050.h
 *
 *  Created on: Mar 26, 2025
 *      Author: talur
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include<stdint.h>
#include<stdio.h>
#include"stm32f4xx_hal.h"



typedef enum
{
	MPU6050_ok,
	MPU6050_err

}MPU6050_status_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;

}mpu6050_accel_data_t;

typedef enum
{
	INT_LEVEL_ACTIVE_HIGH = 0x00,
	INT_LEVEL_ACTIVE_LOW,

}mpu6050_interrupt_config_t;

typedef enum
{
	RAW_RDY_INT = 0x01,
	I2C_MST_INT = 0x08,
	FIFO_OFLOW_INT = 0x10,
	MOT_INT = 0x40,
	ALL_INT = 0xFF

}mpu6050_interrupt_t;

typedef enum {
	DLPF_CFG_260HZ = 0,
	DLPF_CFG_184HZ = 1,
	DLPF_CFG_94HZ = 2,
	DLPF_CFG_44HZ = 3,
	DLPF_CFG_21HZ = 4,
	DLPF_CFG_10HZ = 5,
	DLPF_CFG_5HZ = 6,

}mpu6050_dlpf_config_t;

MPU6050_status_t MPU6050_init(I2C_HandleTypeDef *hi2c,uint8_t i2c_dev_addr);
MPU6050_status_t MPU6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c,uint8_t i2c_dev_addr,mpu6050_accel_data_t *accel_data);
mpu6050_accel_data_t mpu6050_accelerometer_calibration(const mpu6050_accel_data_t *error_offset,mpu6050_accel_data_t *raw_data);
MPU6050_status_t mpu6050_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf \
);

MPU6050_status_t mpu6050_interrupt_config(I2C_HandleTypeDef *hi2c,mpu6050_interrupt_config_t level);
MPU6050_status_t mpu6050_enable_interrupt(I2C_HandleTypeDef *hi2c,mpu6050_interrupt_t interrupt);
MPU6050_status_t mpu6050_disable_interrupt(I2C_HandleTypeDef *hi2c,mpu6050_interrupt_t interrupt);
void mpu6050_interrupt_handle(I2C_HandleTypeDef *hi2c);
void  mpu6050_raw_data_ready_callback(void);
void mpu6050_motion_detection_callback(void);

//MPU6050 reg addresses
#define MPU6050_REG_WHOAMI 		(uint8_t)117
#define MPU6050_REG_PWMGMT_1 	(uint8_t)0x6B
#define MPU6050_REG_ACCEL_START (uint8_t)59
#define MPU6050_REG_CONFIG      (uint8_t)26
#define MPU6050_REG_USER_CTRL	(uint8_t)0x6A
#define MPU6050_REG_BANK_SEL    (uint8_t)0x6D
#define MPU6050_REG_MEM_START_ADDR   0x6E
#define MPU6050_REG_MEM_R_W          0x6F
#define MPU6050_REG_INT_STATUS     0x3A
#define MPU6050_REG_INT_EN         0x38
#define MPU6050_REG_MOT_THR 0x1F
#define MPU6050_REG_MOT_DUR 0x20
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_FIFO_SOURCE_EN 0x23
#define MPU6050_REG_DMP_PROG_ADDR 0x70
#define MPU6050_REG_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_REG_FIFO_COUNT_H 0x72
#define MPU6050_REG_FIFO_COUNT_L 0x73
#define MPU6050_REG_FIFO_R_W 0x74




#endif /* INC_MPU6050_H_ */
