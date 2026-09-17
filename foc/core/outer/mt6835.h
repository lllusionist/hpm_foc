#ifndef __MT6835_H__
#define __MT6835_H__

#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_spi_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include <stdio.h>
#include "spi.h"

// ---- 硬件连接 (已和原理图核对) ----
// PC10 = SPI_SCK, PC11 = SPI_CSN, PC12 = SPI_MISO, PC13 = SPI_MOSI
#define MT6835_SPI              HPM_SPI1
#define MT6835_SPI_SCLK_FREQ    (10000000UL)    // SPI 时钟, MT6835 上限 16MHz
#define MT6835_CS_GPIO          HPM_GPIO0       // CS 用普通 GPIO 控制
#define MT6835_CS_PORT          GPIO_DO_GPIOC
#define MT6835_CS_PIN           (11U)           // PC11
#define MT6835_FRAME_LEN        (6U)            // 一帧 6 字节: A0 03 00 00 00 00
#define MT6835_UPDATE_FREQ      (20000U)        // 处理频率 = PWM 频率

// ---- 换算参数(按自己的电机和磁铁安装改) ----
#define MT6835_ANGLE_OFFSET     (16663688)      // 机械零位偏移, 要实测标定
#define MT6835_POLE_PAIRS       (7U)            // 电机极对数
#define MT6835_SPEED_LPF_COEFF  (0.002f)        // 电角速度一阶低通系数

typedef struct {
    int32_t  angle_raw;         // 21 位原始角度(已减零位)
    float    angle_rad;         // 机械角, rad
    float    elec_angle;        // 电角度, rad
    float    elec_speed;        // 电角速度, rad/s
    float    elec_speed_lpf;    // 滤波后的电角速度, rad/s
    uint32_t error_cnt;         // 传输启动失败次数
} mt6835_data_t;

extern volatile mt6835_data_t MT6835;

void mt6835_init(void);
void mt6835_start_read(void);
void mt6835_update_angle(void);

#endif /* __MT6835_H__ */
