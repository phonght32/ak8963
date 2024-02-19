#include "stdlib.h"
#include "string.h"
#include "ak8963.h"

#define AK8963_WHO_AM_I             0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_XOUT_L               0x03
#define AK8963_XOUT_H               0x04
#define AK8963_YOUT_L               0x05
#define AK8963_YOUT_H               0x06
#define AK8963_ZOUT_L               0x07
#define AK8963_ZOUT_H               0x08
#define AK8963_ST2                  0x09
#define AK8963_CNTL                 0x0A
#define AK8963_RSV                  0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

#define BUFFER_CALIB_DEFAULT        1000        /*!< Default the number of sample data when calibrate */

typedef struct ak8963 {
	ak8963_mode_t  				opr_mode; 					/*!< Operation mode */
	ak8963_mfs_sel_t  			mfs_sel; 			  		/*!< Magnetometer full scale */
	float                       mag_hard_iron_bias_x;       /*!< Magnetometer hard iron bias of x axis */
	float                       mag_hard_iron_bias_y;       /*!< Magnetometer hard iron bias of y axis */
	float                       mag_hard_iron_bias_z;       /*!< Magnetometer hard iron bias of z axis */
	float                       mag_soft_iron_bias_x;       /*!< Magnetometer soft iron bias of x axis */
	float                       mag_soft_iron_bias_y;       /*!< Magnetometer soft iron bias of y axis */
	float                       mag_soft_iron_bias_z;       /*!< Magnetometer soft iron bias of z axis */
	ak8963_func_i2c_recv        i2c_recv;          			/*!< AK8963 read bytes */
	ak8963_func_i2c_send        i2c_send;         			/*!< AK8963 write bytes */
	ak8963_func_delay          	delay;                 		/*!< Delay function */
	float 						mag_scaling_factor;			/*!< Magnetometer scaling factor */
	float  						mag_sens_adj_x; 			/*!< Magnetometer sensitive adjust of x axis */
	float  						mag_sens_adj_y;				/*!< Magnetometer sensitive adjust of y axis */
	float  						mag_sens_adj_z;				/*!< Magnetometer sensitive adjust of z axis */
} ak8963_t;

static err_code_t ak8963_i2c_write_reg(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buf_send[len + 1];

	buf_send[0] = reg_addr;
	memcpy(&buf_send[1], buf, len);
	handle->i2c_send(buf_send, len + 1);

	return ERR_CODE_SUCCESS;
}

static err_code_t ak8963_i2c_read_reg(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	uint8_t buffer[1];

	buffer[0] = reg_addr | 0x80;
	handle->i2c_send(buffer, 1);
	handle->i2c_recv(buf, len);

	return ERR_CODE_SUCCESS;
}

ak8963_handle_t ak8963_init(void)
{
	ak8963_handle_t handle = calloc(1, sizeof(ak8963_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

err_code_t ak8963_set_config(ak8963_handle_t handle, ak8963_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	float mag_scaling_factor;

	switch (config.mfs_sel)
	{
	case AK8963_MFS_14BIT:
		mag_scaling_factor = 10.0f * 4912.0f / 8190.0f;
		break;

	case AK8963_MFS_16BIT:
		mag_scaling_factor = 10.0f * 4912.0f / 32760.0f;
		break;

	default:
		break;
	}

	handle->opr_mode = config.opr_mode;
	handle->mfs_sel = config.mfs_sel;
	handle->mag_hard_iron_bias_x = config.mag_hard_iron_bias_x;
	handle->mag_hard_iron_bias_y = config.mag_hard_iron_bias_y;
	handle->mag_hard_iron_bias_z = config.mag_hard_iron_bias_z;
	handle->mag_soft_iron_bias_x = config.mag_soft_iron_bias_x;
	handle->mag_soft_iron_bias_y = config.mag_soft_iron_bias_y;
	handle->mag_soft_iron_bias_z = config.mag_soft_iron_bias_z;
	handle->i2c_recv = config.i2c_recv;
	handle->i2c_send = config.i2c_send;
	handle->delay = config.delay;
	handle->mag_scaling_factor = mag_scaling_factor;
	handle->mag_sens_adj_x = 0;
	handle->mag_sens_adj_y = 0;
	handle->mag_sens_adj_z = 0;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_config(ak8963_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	/* Power down AK8963 magnetic sensor */
	uint8_t buffer = 0;
	buffer = 0x00;
	ak8963_i2c_write_reg(handle, AK8963_CNTL, &buffer, 1);

	/* Delay 10ms here if necessary */
	handle->delay(10);

	/* Set fuse ROM access mode */
	buffer = 0x0F;
	ak8963_i2c_write_reg(handle, AK8963_CNTL, &buffer, 1);

	/* Delay 10ms here if necessary */
	handle->delay(10);

	/* Power down AK8963 magnetic sensor */
	buffer = 0x00;
	ak8963_i2c_write_reg(handle, AK8963_CNTL, &buffer, 1);

	/* Delay 10ms here if necessary */
	handle->delay(10);

	/* Configure magnetic operation mode and range */
	buffer = 0;
	buffer = handle->opr_mode & 0x0F;
	buffer |= (handle->mfs_sel << 4) & 0x10;
	ak8963_i2c_write_reg(handle, AK8963_CNTL, &buffer, 1);

	/* Read magnetic sensitivity adjustment */
	uint8_t mag_raw_data[3];
	ak8963_i2c_read_reg(handle, AK8963_ASAX, mag_raw_data, 3);

	/* Update magnetometer sensitive adjust */
	handle->mag_sens_adj_x = (float)(mag_raw_data[0] - 128) / 256.0f + 1.0f;
	handle->mag_sens_adj_y = (float)(mag_raw_data[1] - 128) / 256.0f + 1.0f;
	handle->mag_sens_adj_z = (float)(mag_raw_data[2] - 128) / 256.0f + 1.0f;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_get_mag_raw(ak8963_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (raw_x == NULL) || (raw_y == NULL) || (raw_z == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t mag_raw_data[7];
	ak8963_i2c_read_reg(handle, AK8963_XOUT_L, mag_raw_data, 7);

	if ((mag_raw_data[6] & 0x08)) {
		return ERR_CODE_FAIL;
	}

	*raw_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
	*raw_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
	*raw_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_get_mag_calib(ak8963_handle_t handle, float *calib_x, float *calib_y, float *calib_z)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (calib_x == NULL) || (calib_y == NULL) || (calib_z == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t mag_raw_data[7];
	int16_t raw_x = 0, raw_y = 0, raw_z = 0;

	ak8963_i2c_read_reg(handle, AK8963_XOUT_L, mag_raw_data, 7);

	if ((mag_raw_data[6] & 0x08)) {
		return ERR_CODE_FAIL;
	}

	raw_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
	raw_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
	raw_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

	*calib_x = ((float)raw_x * handle->mag_sens_adj_x - handle->mag_hard_iron_bias_x / handle->mag_scaling_factor) * handle->mag_soft_iron_bias_x;
	*calib_y = ((float)raw_y * handle->mag_sens_adj_y - handle->mag_hard_iron_bias_y / handle->mag_scaling_factor) * handle->mag_soft_iron_bias_y;
	*calib_z = ((float)raw_z * handle->mag_sens_adj_z - handle->mag_hard_iron_bias_z / handle->mag_scaling_factor) * handle->mag_soft_iron_bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_get_mag_scale(ak8963_handle_t handle, float *scale_x, float *scale_y, float *scale_z)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (scale_x == NULL) || (scale_y == NULL) || (scale_z == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	uint8_t mag_raw_data[7];
	int16_t raw_x = 0, raw_y = 0, raw_z = 0;

	ak8963_i2c_read_reg(handle, AK8963_XOUT_L, mag_raw_data, 7);

	if ((mag_raw_data[6] & 0x08)) {
		return ERR_CODE_FAIL;
	}

	raw_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
	raw_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
	raw_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

	*scale_x = ((float)raw_x * handle->mag_sens_adj_x * handle->mag_scaling_factor - handle->mag_hard_iron_bias_x) * handle->mag_soft_iron_bias_x;
	*scale_y = ((float)raw_y * handle->mag_sens_adj_y * handle->mag_scaling_factor - handle->mag_hard_iron_bias_y) * handle->mag_soft_iron_bias_y;
	*scale_z = ((float)raw_z * handle->mag_sens_adj_z * handle->mag_scaling_factor - handle->mag_hard_iron_bias_z) * handle->mag_soft_iron_bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_set_mag_hard_iron_bias(ak8963_handle_t handle, float bias_x, float bias_y, float bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->mag_hard_iron_bias_x = bias_x;
	handle->mag_hard_iron_bias_y = bias_y;
	handle->mag_hard_iron_bias_z = bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_set_mag_soft_iron_bias(ak8963_handle_t handle, float bias_x, float bias_y, float bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->mag_soft_iron_bias_x = bias_x;
	handle->mag_soft_iron_bias_y = bias_y;
	handle->mag_soft_iron_bias_z = bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_get_mag_hard_iron_bias(ak8963_handle_t handle, float *bias_x, float *bias_y, float *bias_z)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (bias_x == NULL) || (bias_y == NULL) || (bias_z == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	*bias_x = handle->mag_hard_iron_bias_x;
	*bias_y = handle->mag_hard_iron_bias_y;
	*bias_z = handle->mag_hard_iron_bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t ak8963_get_mag_soft_iron_bias(ak8963_handle_t handle, float *bias_x, float *bias_y, float *bias_z)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (bias_x == NULL) || (bias_y == NULL) || (bias_z == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	*bias_x = handle->mag_soft_iron_bias_x;
	*bias_y = handle->mag_soft_iron_bias_y;
	*bias_z = handle->mag_soft_iron_bias_z;

	return ERR_CODE_SUCCESS;
}
