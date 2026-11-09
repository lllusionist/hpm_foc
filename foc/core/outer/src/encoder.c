#include "encoder.h"
#include "config.h"

volatile tEncoder encoder;

/* 收发缓冲: 必须放非 cache 段, DMA 才能直接读写 */
ATTR_PLACE_AT_NONCACHEABLE static uint8_t encoder_tx_buf[ENCODER_FRAME_LEN];
ATTR_PLACE_AT_NONCACHEABLE static uint8_t encoder_rx_buf[ENCODER_FRAME_LEN];

static volatile bool rxdma_done;  /* 收 DMA 完成 */
static volatile bool txdma_done;  /* 发 DMA 完成 */
static volatile bool spi_busy;	  /* 有一帧正在传 */
static volatile bool frame_ready; /* 收到完整一帧, 等处理 */

/* CS 控制: 0 = 选中(拉低), 1 = 释放(拉高) */
static void encoder_cs(uint8_t level)
{
	gpio_write_pin(ENCODER_CS_GPIO, ENCODER_CS_PORT, ENCODER_CS_PIN, level);
}

/* 填发送缓冲: A0 = 读角度命令, 03 = 后面跟着读 3 字节 */
static void encoder_prepare_tx(void)
{
	encoder_tx_buf[0] = 0xA0;
	encoder_tx_buf[1] = 0x03;
	encoder_tx_buf[2] = 0x00;
	encoder_tx_buf[3] = 0x00;
	encoder_tx_buf[4] = 0x00;
	encoder_tx_buf[5] = 0x00;

	for (uint8_t i = 0; i < ENCODER_FRAME_LEN; i++)
		encoder_rx_buf[i] = 0;
}

static void encoder_try_finish(void)
{
	if (spi_busy && rxdma_done && txdma_done) {
		encoder_cs(1); /* 拉高 CS, 结束这一帧 */
		frame_ready = true;
		spi_busy = false;
	}
}

static void encoder_rxdma_callback(uint32_t channel)
{
	(void)channel;
	rxdma_done = true;
	encoder_try_finish();
}

static void encoder_txdma_callback(uint32_t channel)
{
	(void)channel;
	txdma_done = true;
	encoder_try_finish();
}

/* 复位角度和 PLL; 增益按调用频率折算 */
void ENCODER_reset(void)
{
	float bw = ENCODER_PLL_BW_HZ * 6.2831853f;

	encoder.raw = 0;
	encoder.count_in_cpr = 0;
	encoder.count_in_cpr_prev = 0;
	encoder.count_aligned = 0;
	encoder.shadow_count = 0;

	encoder.pos = 0;
	encoder.vel = 0;
	encoder.phase = 0;
	encoder.phase_vel = 0;

	encoder.pll_pos = 0;
	encoder.pll_vel = 0;
	encoder.pll_kp = 2.0f * bw;
	encoder.pll_ki = 0.25f * encoder.pll_kp * encoder.pll_kp;
	encoder.snap_threshold = ENCODER_PLL_SNAP_VEL;
	encoder.need_init = 20;

	encoder.error_cnt = 0;
}

void ENCODER_init(void)
{
	spi_initialize_config_t init_config;

	HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_GPIO_C_11;
	HPM_IOC->PAD[IOC_PAD_PC11].PAD_CTL =
	    IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) |
	    IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) |
	    IOC_PAD_PAD_CTL_PRS_SET(1);
	HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL =
	    IOC_PC10_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
	HPM_IOC->PAD[IOC_PAD_PC10].PAD_CTL =
	    IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);
	HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_SPI1_MISO;
	HPM_IOC->PAD[IOC_PAD_PC12].PAD_CTL =
	    IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);
	HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PC13_FUNC_CTL_SPI1_MOSI;
	HPM_IOC->PAD[IOC_PAD_PC13].PAD_CTL =
	    IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);

	/* CS 先拉高, 免得初始化过程中误选中 */
	gpio_set_pin_output_with_initial(ENCODER_CS_GPIO, ENCODER_CS_PORT,
					 ENCODER_CS_PIN, 1);

	clock_add_to_group(clock_spi1, 0);

	hpm_spi_get_default_init_config(&init_config);
	init_config.direction = spi_msb_first;
	init_config.mode = spi_master_mode;
	init_config.clk_polarity = spi_sclk_high_idle;		  /* CPOL=1 */
	init_config.clk_phase = spi_sclk_sampling_even_clk_edges; /* CPHA=1 */
	init_config.data_len = 8;

	if (hpm_spi_initialize(ENCODER_SPI, &init_config) != status_success) {
		printf("encoder: spi init fail\n");
		while (1) {
		}
	}
	if (hpm_spi_set_sclk_frequency(ENCODER_SPI, ENCODER_SPI_SCLK_FREQ) !=
	    status_success) {
		printf("encoder: set sclk fail\n");
		while (1) {
		}
	}

	/* 必须先调 dma_mgr_init() */
	if (hpm_spi_dma_mgr_install_callback(
		ENCODER_SPI, encoder_txdma_callback, encoder_rxdma_callback) !=
	    status_success) {
		printf("encoder: install dma callback fail\n");
		while (1) {
		}
	}

	encoder_prepare_tx();
	ENCODER_reset();
}

/* 启动一帧非阻塞读, 传完由 DMA 回调收尾 */
void ENCODER_start_read(void)
{
	hpm_stat_t stat;

	if (spi_busy)
		return; /* 上一帧还没传完, 这一拍跳过 */

	rxdma_done = false;
	txdma_done = false;
	spi_busy = true;

	encoder_cs(0); /* 拉低 CS 选中芯片 */

	stat = hpm_spi_transmit_receive_nonblocking(
	    ENCODER_SPI, encoder_tx_buf, encoder_rx_buf, ENCODER_FRAME_LEN);

	if (stat != status_success) {
		encoder_cs(
		    1); /* 启动失败要立刻放开 CS, 否则后面一直卡在 busy */
		spi_busy = false;
		encoder.error_cnt++;
	}
}

/* 处理上一帧; 角度和速度都走 PLL */
void ENCODER_loop(void)
{
	uint32_t raw, corrected;
	int32_t delta;
	float period = 1.0f / (float)ENCODER_UPDATE_FREQ;
	float count, pos_err;

	if (!frame_ready)
		return;
	frame_ready = false;

	/* 21 位角度在高 21 位 */
	raw = ((uint32_t)encoder_rx_buf[2] << 16) |
	      ((uint32_t)encoder_rx_buf[3] << 8) |
	      ((uint32_t)encoder_rx_buf[4] & 0xF8U);
	encoder.raw = raw;

	if (Config.encoder_dir == 1)
		corrected = raw;
	else
		corrected = ENCODER_MASK - raw;
	encoder.count_in_cpr = corrected;

	/* 前几拍只对齐, 不参与累加和 PLL */
	if (encoder.need_init > 0U) {
		encoder.count_in_cpr_prev = corrected;
		encoder.pll_pos = (float)corrected;
		encoder.pll_vel = 0;
		encoder.need_init--;
		return;
	}

	/* 单圈差分做过零修正再累加 */
	delta = (int32_t)corrected - (int32_t)encoder.count_in_cpr_prev;
	if (delta > (int32_t)ENCODER_CPR_HALF)
		delta -= (int32_t)ENCODER_CPR;
	else if (delta < -(int32_t)ENCODER_CPR_HALF)
		delta += (int32_t)ENCODER_CPR;
	/* 帧合理性检查: MT6835 帧里没有可用的校验, SPI 受开关干扰时会读到
	 * 位翻转的假角度. 一个 5kHz 周期内机械角最大变化按 10rps 算 =
	 * 1/500 圈, 超过必然是坏帧 -> 丢弃. 坏帧计数定期打印便于定位干扰. */
	if (abs(delta) > (int32_t)(ENCODER_CPR / 500)) {
		encoder.bad_frame_cnt++;

		if ((encoder.bad_frame_cnt % 200U) == 1U) {
			printf("enc: bad frame #%lu (delta=%ld cnt)\n",
			       (unsigned long)encoder.bad_frame_cnt,
			       (long)delta);
		}

		return; /* 保留上一拍角度, 不累加、不喂 PLL */
	}

	encoder.shadow_count += (int64_t)delta;
	encoder.count_in_cpr_prev = corrected;

	/* 减零位 + 偏移表后再算角度 (校准结果) */
	uint32_t lut_index =
	    (uint32_t)(((uint64_t)corrected * OFFSET_LUT_NUM) / ENCODER_CPR);
	int32_t aligned_count = (int32_t)corrected - Config.encoder_offset -
				Config.offset_lut[lut_index];
	encoder.count_aligned = (uint32_t)aligned_count & ENCODER_MASK;
	count = (float)encoder.count_aligned;
	/* 多圈机械位置 (圈), 用于位置环/回零 */
	encoder.pos = (float)encoder.shadow_count / ENCODER_CPR_F;
	encoder.phase =
	    (float)((encoder.count_aligned * (uint32_t)Config.pole_pairs) &
		    ENCODER_MASK) *
	    (6.2831853f / ENCODER_CPR_F);

	/* 误差归一化到半圈 */
	pos_err = count - encoder.pll_pos;
	if (pos_err > (float)ENCODER_CPR_HALF)
		pos_err -= ENCODER_CPR_F;
	else if (pos_err < -(float)ENCODER_CPR_HALF)
		pos_err += ENCODER_CPR_F;

	encoder.pll_pos +=
	    (encoder.pll_vel + encoder.pll_kp * pos_err) * period;
	encoder.pll_vel += (encoder.pll_ki * pos_err) * period;

	if (encoder.pll_pos >= ENCODER_CPR_F)
		encoder.pll_pos -= ENCODER_CPR_F;
	else if (encoder.pll_pos < 0.0f)
		encoder.pll_pos += ENCODER_CPR_F;

	/* 静止死区, 防漂 */
	if ((encoder.pll_vel < encoder.snap_threshold) &&
	    (encoder.pll_vel > -encoder.snap_threshold)) {
		encoder.pll_vel = 0;
	}

	encoder.vel = encoder.pll_vel / ENCODER_CPR_F;
	encoder.phase_vel = encoder.vel * 6.2831853f * (float)Config.pole_pairs;
}

/* 先发起一帧再处理上一帧, 两者重叠所以周期是 2 个 tick */
static void Encoder_task(void *argument)
{
	TickType_t last = xTaskGetTickCount();

	(void)argument;

	for (;;) {
		vTaskDelayUntil(&last, ENCODER_HALF_TICKS);
		ENCODER_start_read();
		vTaskDelayUntil(&last, ENCODER_HALF_TICKS);
		ENCODER_loop();
	}
}

void encoder_app_start(void)
{
	if (xTaskCreate(Encoder_task, "enc", 512, NULL, 20, NULL) != pdPASS) {
		printf("encoder task create fail\n");
		while (1) {
		}
	}
}
