/*
 * MPU6050.c
 *
 *  Created on: Mar 26, 2025
 *      Author: talur
 */

#include <MPU6050.h>

#define I2C_TIMEOUT 500UL

static uint8_t MPU6050_i2c_addr;




MPU6050_status_t MPU6050_read_byte(I2C_HandleTypeDef *hi2c,uint8_t reg_addr,uint8_t *data)
{
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c,MPU6050_i2c_addr<<1,reg_addr,1,data,1,I2C_TIMEOUT);

	return (status == HAL_OK) ? MPU6050_ok : MPU6050_err;
}



MPU6050_status_t MPU6050_read(I2C_HandleTypeDef *hi2c,uint8_t reg_base_addr,uint8_t *buffer,uint32_t nbytes)
{
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c,MPU6050_i2c_addr<<1,reg_base_addr,1,buffer,nbytes,I2C_TIMEOUT);

	return (status == HAL_OK) ? MPU6050_ok : MPU6050_err;
}

MPU6050_status_t MPU6050_write_byte(I2C_HandleTypeDef *hi2c,uint8_t reg_addr,uint8_t data)
{
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c,MPU6050_i2c_addr<<1,reg_addr,1,&data,1,I2C_TIMEOUT);

	return (status == HAL_OK) ? MPU6050_ok : MPU6050_err;
}


MPU6050_status_t MPU6050_init(I2C_HandleTypeDef *hi2c,uint8_t i2c_dev_addr)
{

	MPU6050_i2c_addr = i2c_dev_addr;

	//1.read 1 byte from sensor
	uint8_t read_byte = 0;
	if(MPU6050_read_byte(hi2c,MPU6050_REG_WHOAMI,&read_byte)!= MPU6050_ok)
	{
		return MPU6050_err;
	}

	if(read_byte == 0x68 || read_byte == 0x98)
	{
		printf("valid MPU6050 found at address %x\n",MPU6050_i2c_addr);
	}
	else
	{
		printf("Invalid MPU6050 found at address %x\n",MPU6050_i2c_addr);
		return MPU6050_err;

	}

	uint8_t data = 0x00;
	if(MPU6050_write_byte(hi2c,MPU6050_REG_PWMGMT_1,data)!= MPU6050_ok)
	{
		return MPU6050_err;
	}

	return MPU6050_ok;
}





MPU6050_status_t MPU6050_read_accelerometer_data(I2C_HandleTypeDef *hi2c,uint8_t i2c_dev_addr,mpu6050_accel_data_t *accel_data)
{
	uint8_t raw_data[6];
	MPU6050_status_t status = MPU6050_read(hi2c,MPU6050_REG_ACCEL_START,raw_data,sizeof(raw_data));

	if(status != MPU6050_ok)
	{
		return status;
	}

	accel_data->x = (int16_t)(raw_data[0]<< 8 | raw_data[1]);
	accel_data->y = (int16_t)(raw_data[2]<< 8 | raw_data[3]);
	accel_data->z = (int16_t)(raw_data[4]<< 8 | raw_data[5]);

	return MPU6050_ok;
}


mpu6050_accel_data_t mpu6050_accelerometer_calibration(const mpu6050_accel_data_t *error_offset,mpu6050_accel_data_t *raw_data)
{
	mpu6050_accel_data_t accel_calibrated;

	accel_calibrated.x = raw_data->x - error_offset->x;
	accel_calibrated.y = raw_data->y - error_offset->y;
	accel_calibrated.z = raw_data->z - error_offset->z;

	return accel_calibrated;

}



MPU6050_status_t mpu6050_interrupt_config(I2C_HandleTypeDef *hi2c,mpu6050_interrupt_config_t level)
{
	uint8_t int_cfg = 0;

	if(MPU6050_read_byte(hi2c,MPU6050_REG_INT_PIN_CFG,&int_cfg)!= MPU6050_ok)
	{
		return MPU6050_err;
	}



	int_cfg &= ~0x80;
	int_cfg |= (uint8_t)level;

	//write the updated configuration back to the register
	return MPU6050_write_byte(hi2c,MPU6050_REG_INT_PIN_CFG,int_cfg);
}

MPU6050_status_t mpu6050_enable_interrupt(I2C_HandleTypeDef *hi2c,mpu6050_interrupt_t interrupt)
{
	uint8_t current_int_settings = 0;
			if(MPU6050_read_byte(hi2c,MPU6050_REG_INT_EN,&current_int_settings) != MPU6050_ok)
			{
				return MPU6050_err;
			}

			current_int_settings |= (uint8_t)interrupt;
			return MPU6050_write_byte(hi2c,MPU6050_REG_INT_EN,current_int_settings);
}

MPU6050_status_t mpu6050_disable_interrupt(I2C_HandleTypeDef *hi2c,mpu6050_interrupt_t interrupt)
{
	uint8_t current_int_settings = 0;
		if(interrupt != (uint8_t)ALL_INT)
		{
			MPU6050_read_byte(hi2c,MPU6050_REG_INT_EN,&current_int_settings);
		}

		current_int_settings &= ~interrupt;
		return MPU6050_write_byte(hi2c,MPU6050_REG_INT_EN,current_int_settings);
}

MPU6050_status_t mpu6050_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf \
)
{

	uint8_t value = 0;

	if (MPU6050_read_byte(hi2c, MPU6050_REG_CONFIG, &value) != MPU6050_ok){
		return MPU6050_err;
	}

	value &= ~(0x7);
	value |= (uint8_t)dlpf;
	if (MPU6050_write_byte(hi2c, MPU6050_REG_CONFIG, value) != MPU6050_ok){
		return MPU6050_err;
	}

	return MPU6050_ok;
}




static MPU6050_status_t get_interrupt_status(I2C_HandleTypeDef *hi2c, uint8_t *data) {
	//read the interrupt status register MPU6050_REG_INT_STATUS
	return MPU6050_read_byte(hi2c, MPU6050_REG_INT_STATUS, data);
}

static MPU6050_status_t get_interrupt_setting(I2C_HandleTypeDef *hi2c, uint8_t *data) {
	//read the interrupt status register MPU6050_REG_INT_STATUS
	return MPU6050_read_byte(hi2c, MPU6050_REG_INT_EN, data);
}

void mpu6050_interrupt_handle(I2C_HandleTypeDef *hi2c)
{
	//read the interrupt status register of the sensor
	uint8_t int_status = 0;
	uint8_t int_en_status;
	uint8_t int_settings = 0;
	get_interrupt_status(hi2c,&int_status);
	get_interrupt_setting(hi2c,&int_status);

	if((int_settings & MOT_INT) && (int_status & MOT_INT))
	{
		mpu6050_motion_detection_callback();
	}
	else if((int_settings & RAW_RDY_INT)&&(int_status & RAW_RDY_INT))
	{
		mpu6050_raw_data_ready_callback();
	}
	else
	{

	}
}


__weak mpu6050_motion_detection_callback(void)
{

}

__weak mpu6050_raw_data_ready_callback(void)
{

}
