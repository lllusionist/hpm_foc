#include "encoder.h"
#include "config.h"

volatile tEncoder encoder;

/* DMA 缓冲区必须位于非缓存区，供 DMA 直接读写。 */
ATTR_PLACE_AT_NONCACHEABLE static uint8_t encoder_tx_buf[ENCODER_FRAME_LEN];
ATTR_PLACE_AT_NONCACHEABLE static uint8_t encoder_rx_buf[ENCODER_FRAME_LEN];

static volatile bool rxdma_done;  /* 收 DMA 完成 */
static volatile bool txdma_done;  /* 发 DMA 完成 */
static volatile bool spi_busy;	  /* 有一帧正在传 */
static volatile bool frame_ready; /* 收到完整一帧, 等处理 */

/* CS 拉低时选中编码器。 */
static void encoder_cs(uint8_t level)
{
	gpio_write_pin(ENCODER_CS_GPIO, ENCODER_CS_PORT, ENCODER_CS_PIN, level);
}

/* 准备读取角度的 SPI 命令。 */
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

/* 重置编码器状态，并按更新频率设置 PLL 增益。 */
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
	encoder.bad_run = 0;
	encoder.bad_frame_cnt = 0;
	encoder.frame_cnt = 0;
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

/* 启动非阻塞 SPI 读取，由 DMA 回调完成本帧处理。 */
void ENCODER_start_read(void)
{
	hpm_stat_t stat;

	if (spi_busy) {
		/* 上一帧 DMA 回调没来: 计数并看是否超时 */
		encoder.error_cnt++;
		if (encoder.error_cnt >= ENCODER_SPI_TIMEOUT_FRAMES) {
			/* DMA 卡死: 强制释放, 下一拍重新开始 */
			spi_busy = false;
			encoder_cs(1);
			encoder.bad_frame_cnt +=
			    ENCODER_SPI_TIMEOUT_FRAMES;
		}
		return;
	}

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

/* 方向修正后的单圈值 -> 扣除标定零偏和偏移表后的单圈值.
 * PLL 跟踪的是对齐后的计数, 初始化/重同步必须用同一坐标域 */
static uint32_t encoder_align(uint32_t corrected)
{
	uint32_t lut_index =
	    (uint32_t)(((uint64_t)corrected * OFFSET_LUT_NUM) / ENCODER_CPR);
	int32_t aligned = (int32_t)corrected - Config.encoder_offset -
			  Config.offset_lut[lut_index];

	return (uint32_t)aligned & ENCODER_MASK;
}

/* 解析最新帧并更新位置与速度。 */
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
		encoder.pll_pos = (float)encoder_align(corrected);
		encoder.pll_vel = 0;
		encoder.need_init--;
		return;
	}

	/* 每秒统计一次坏帧比例。 */
	encoder.frame_cnt++;
	{
		static uint32_t bad_at_last;

		if ((encoder.frame_cnt % 5000U) == 0U) {
			uint32_t bad = encoder.bad_frame_cnt - bad_at_last;

			if (bad > 0U)
				printf("enc: bad %lu/5000 frames (%lu%%)\n",
				       (unsigned long)bad,
				       (unsigned long)(bad / 50U));
			bad_at_last = encoder.bad_frame_cnt;
		}
	}

	/* 修正单圈过零差分并累加多圈位置。 */
	delta = (int32_t)corrected - (int32_t)encoder.count_in_cpr_prev;
	if (delta > (int32_t)ENCODER_CPR_HALF)
		delta -= (int32_t)ENCODER_CPR;
	else if (delta < -(int32_t)ENCODER_CPR_HALF)
		delta += (int32_t)ENCODER_CPR;
	/* 坏帧阈值按配置最高转速留出余量，以过滤 SPI 位错误。 */
	float max_track_rps = Config.velocity_limit * 3.0f;
	if (max_track_rps < 30.0f)
		max_track_rps = 30.0f;
	/* 标定提高速度上限时仍限制阈值，避免关闭坏帧过滤。 */
	if (max_track_rps > 150.0f)
		max_track_rps = 150.0f;
	int32_t max_delta = (int32_t)(ENCODER_CPR_F * max_track_rps /
				      (float)ENCODER_UPDATE_FREQ);
	if (abs(delta) > max_delta) {
		encoder.bad_frame_cnt++; /* 总帧数已在上面累加 */

		/* 连续坏帧后重新同步，恢复位置跟踪。 */
		if (++encoder.bad_run >= 5U) {
			encoder.bad_run = 0;
			encoder.count_in_cpr_prev = corrected;
			encoder.pll_pos = (float)encoder_align(corrected);
		}

		return; /* 保留上一拍角度, 不累加、不喂 PLL */
	}
	encoder.bad_run = 0;

	encoder.shadow_count += (int64_t)delta;
	encoder.count_in_cpr_prev = corrected;

	/* 应用标定零偏和角度修正表。 */
	encoder.count_aligned = encoder_align(corrected);
	count = (float)encoder.count_aligned;
	/* 多圈机械位置，单位为转。 */
	encoder.pos = (float)encoder.shadow_count / ENCODER_CPR_F;
	encoder.phase =
	    (float)((encoder.count_aligned * (uint32_t)Config.pole_pairs) &
		    ENCODER_MASK) *
	    (6.2831853f / ENCODER_CPR_F);

	/* 将 PLL 位置误差归一化到最短单圈差值。 */
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

	/* 静止时抑制小幅速度估计，避免零速漂移。 */
	if ((encoder.pll_vel < encoder.snap_threshold) &&
	    (encoder.pll_vel > -encoder.snap_threshold)) {
		encoder.pll_vel = 0;
	}

	encoder.vel = encoder.pll_vel / ENCODER_CPR_F;
	encoder.phase_vel = encoder.vel * 6.2831853f * (float)Config.pole_pairs;
	encoder.phase_seq++;
}

/* SPI 传输与上一帧处理重叠，按设定频率更新。 */
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
