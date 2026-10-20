/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __UTIL_H__
#define __UTIL_H__

#include "hpm_common.h"
#include <math.h>

#ifdef M_PI
#undef M_PI
#endif
#define M_PI (3.14159265358f)

#define M_2PI (6.28318530716f)
#define ONE_BY_SQRT3 (0.57735026919f)
#define TWO_BY_SQRT3 (2.0f * 0.57735026919f)
#define SQRT3_BY_2 (0.86602540378f)

#define SQ(x) ((x) * (x))
#define ABS(x) ((x) > 0 ? (x) : -(x))

#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif
#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))
#define FLOAT_EQU(floatA, floatB) ((ABS((floatA) - (floatB))) < 0.000001f)
#define UTILS_LP_FAST(value, sample, filter_constant) \
	(value -= (filter_constant) * ((value) - (sample)))

/* 注意: 这里不能用 extern inline —— 定义只在 util.c 里,
 * 其它文件只看到声明会报 "inline function declared but never defined" */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha,
		      float *Ibeta);
void park_transform(float Ialpha, float Ibeta, float Theta, float *Id,
		    float *Iq);
void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha,
		  float *mod_beta);
int SVPWM(float alpha, float beta, volatile float *duty_a,
	  volatile float *duty_b, volatile float *duty_c);

float sin_f32(float x);
float cos_f32(float x);
/* 名字不能叫 crc32：SDK 的 utils/hpm_crc32.c 已经定义了同名全局函数，
 * 链接时会报 "multiply defined global symbol crc32"。改名避免冲突。 */
uint32_t util_crc32(const uint8_t *data, uint32_t size);

int uint32_to_data(uint32_t val, uint8_t *data);
int int32_to_data(int32_t val, uint8_t *data);
int uint16_to_data(uint16_t val, uint8_t *data);
int int16_to_data(int16_t val, uint8_t *data);
int float_to_data(float val, uint8_t *data);

uint32_t data_to_uint32(uint8_t *data);
int32_t data_to_int32(uint8_t *data);
uint16_t data_to_uint16(uint8_t *data);
int16_t data_to_int16(uint8_t *data);
float data_to_float(uint8_t *data);

#endif
