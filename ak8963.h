// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __AK8963_H__
#define __AK8963_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

typedef err_code_t (*ak8963_func_i2c_recv)(uint8_t *buf_recv, uint16_t len, uint32_t timeout_ms);
typedef err_code_t (*ak8963_func_i2c_send)(uint8_t *buf_send, uint16_t len, uint32_t timeout_ms);
typedef void (*ak8963_func_delay)(uint32_t ms);

/**
 * @brief   Handle selection.
 */
typedef struct ak8963 *ak8963_handle_t;

/**
 * @brief   Mode selection.
 */
typedef enum {
	AK8963_MODE_PWR_DOWN = 0x00,                /*!< AK8963 mode power down */
	AK8963_MODE_SINGLE_MEASUREMENT = 0x01,      /*!< AK8963 mode single measurement */
	AK8963_MODE_CONT_MEASUREMENT_1 = 0x02,      /*!< AK8963 mode continous measurement 1 */
	AK8963_MODE_EXT_TRIG_MEASUREMENT = 0x04,    /*!< AK8963 mode external trigger measurement */
	AK8963_MODE_CONT_MEASUREMENT_2 = 0x06,      /*!< AK8963 mode continous measurement 2 */
	AK8963_MODE_SELF_TEST = 0x08,               /*!< AK8963 mode self test */
	AK8963_MODE_FUSE_ROM_ACCESS = 0x0F,         /*!< AK8963 mode fuse ROM access */
	AK8963_MODE_MAX
} ak8963_mode_t;

/**
 * @brief   Magnetometer full scale.
 */
typedef enum {
	AK8963_MFS_14BIT = 0,                       /*!< Magnetometer 14 bit resolution  */
	AK8963_MFS_16BIT,                           /*!< Magnetometer 16 bit resolution  */
	AK8963_MFS_MAX
} ak8963_mfs_sel_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
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
} ak8963_cfg_t;

/*
 * @brief   Initialize AK8963 with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
ak8963_handle_t ak8963_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_set_config(ak8963_handle_t handle, ak8963_cfg_t config);

/*
 * @brief   Configure AK8963 to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_config(ak8963_handle_t handle);

/*
 * @brief   Get magnetometer raw value.
 *
 * @param   handle Handle structure.
 * @param   raw_x Raw value x axis.
 * @param   raw_y Raw value y axis.
 * @param   raw_z Raw value z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_get_mag_raw(ak8963_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get magnetometer calibrated data.
 *
 * @param   handle Handle structure.
 * @param   calib_x Calibrated data x axis.
 * @param   calib_y Calibrated data y axis.
 * @param   calib_z Calibrated data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_get_mag_calib(ak8963_handle_t handle, float *calib_x, float *calib_y, float *calib_z);

/*
 * @brief   Get magnetometer scaled data.
 *
 * @param   handle Handle structure.
 * @param   scale_x Scaled data x axis.
 * @param   scale_y Scaled data y axis.
 * @param   scale_z Scaled data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_get_mag_scale(ak8963_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Set magnetometer hard iron bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_set_mag_hard_iron_bias(ak8963_handle_t handle, float bias_x, float bias_y, float bias_z);

/*
 * @brief   Set magnetometer soft iron bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_set_mag_soft_iron_bias(ak8963_handle_t handle, float bias_x, float bias_y, float bias_z);

/*
 * @brief   Get magnetometer hard iron bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_get_mag_hard_iron_bias(ak8963_handle_t handle, float *bias_x, float *bias_y, float *bias_z);

/*
 * @brief   Get magnetometer soft iron bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t ak8963_get_mag_soft_iron_bias(ak8963_handle_t handle, float *bias_x, float *bias_y, float *bias_z);


#ifdef __cplusplus
}
#endif

#endif /* __AK8963_H__ */