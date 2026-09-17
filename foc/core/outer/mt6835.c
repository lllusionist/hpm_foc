#include "mt6835.h"

volatile mt6835_data_t MT6835;

// 收发缓冲: 必须放非 cache 段, DMA 才能直接读写
ATTR_PLACE_AT_NONCACHEABLE static uint8_t mt6835_tx_buf[MT6835_FRAME_LEN];
ATTR_PLACE_AT_NONCACHEABLE static uint8_t mt6835_rx_buf[MT6835_FRAME_LEN];

static volatile bool rxdma_done;        // 收 DMA 完成
static volatile bool txdma_done;        // 发 DMA 完成
static volatile bool spi_busy;          // 有一帧正在传
static volatile bool frame_ready;       // 收到完整一帧, 等处理

// CS 控制: 0 = 选中(拉低), 1 = 释放(拉高)
static void mt6835_cs(uint8_t level)
{
    gpio_write_pin(MT6835_CS_GPIO, MT6835_CS_PORT, MT6835_CS_PIN, level);
}

// 填发送缓冲: A0 = 读角度命令, 03 = 后面跟着读 3 字节
static void mt6835_prepare_tx(void)
{
    mt6835_tx_buf[0] = 0xA0;
    mt6835_tx_buf[1] = 0x03;
    mt6835_tx_buf[2] = 0x00;
    mt6835_tx_buf[3] = 0x00;
    mt6835_tx_buf[4] = 0x00;
    mt6835_tx_buf[5] = 0x00;

    for (uint8_t i = 0; i < MT6835_FRAME_LEN; i++) {
        mt6835_rx_buf[i] = 0;
    }
}

// 收发两个 DMA 都完成, 才算这一帧结束
static void mt6835_try_finish(void)
{
    if (spi_busy && rxdma_done && txdma_done) {
        mt6835_cs(1);               // 拉高 CS, 结束这一帧
        frame_ready = true;
        spi_busy = false;
    }
}

static void mt6835_rxdma_callback(uint32_t channel)
{
    (void)channel;
    rxdma_done = true;
    mt6835_try_finish();
}

static void mt6835_txdma_callback(uint32_t channel)
{
    (void)channel;
    txdma_done = true;
    mt6835_try_finish();
}

void mt6835_init(void)
{
    spi_initialize_config_t init_config;

    // ---- 1. IO ----
    // PC11 不接 SPI 的硬件 CS, 改成普通 GPIO: 组件内部按帧手动拉, 所以要自己控制
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_GPIO_C_11;
    HPM_IOC->PAD[IOC_PAD_PC11].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PRS_SET(1);
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PC10_FUNC_CTL_SPI1_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC10].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_SPI1_MISO;
    HPM_IOC->PAD[IOC_PAD_PC12].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PC13_FUNC_CTL_SPI1_MOSI;
    HPM_IOC->PAD[IOC_PAD_PC13].PAD_CTL = IOC_PAD_PAD_CTL_SR_SET(1) | IOC_PAD_PAD_CTL_SPD_SET(3);

    // CS 先输出高(未选中), 免得初始化过程中芯片被误选中
    gpio_set_pin_output_with_initial(MT6835_CS_GPIO, MT6835_CS_PORT, MT6835_CS_PIN, 1);

    // ---- 2. SPI 外设 ----
    clock_add_to_group(clock_spi1, 0);

    hpm_spi_get_default_init_config(&init_config);
    init_config.direction    = spi_msb_first;
    init_config.mode         = spi_master_mode;
    init_config.clk_polarity = spi_sclk_high_idle;                  // CPOL=1
    init_config.clk_phase    = spi_sclk_sampling_even_clk_edges;    // CPHA=1, 合起来就是 Mode 3
    init_config.data_len     = 8;                                   // MT6835 按字节收发

    if (hpm_spi_initialize(MT6835_SPI, &init_config) != status_success) {
        printf("mt6835: spi init fail\n");
        while (1) {
        }
    }
    if (hpm_spi_set_sclk_frequency(MT6835_SPI, MT6835_SPI_SCLK_FREQ) != status_success) {
        printf("mt6835: set sclk fail\n");
        while (1) {
        }
    }

    // ---- 3. DMA 回调 ----
    // 这一步会去申请 DMA 通道, 所以必须先调用 dma_mgr_init()
    if (hpm_spi_dma_mgr_install_callback(MT6835_SPI, mt6835_txdma_callback, mt6835_rxdma_callback) != status_success) {
        printf("mt6835: install dma callback fail\n");
        while (1) {
        }
    }

    mt6835_prepare_tx();
}

// 启动一帧非阻塞读; 传完由 DMA 回调收尾, 函数本身立刻返回
void mt6835_start_read(void)
{
    hpm_stat_t stat;

    if (spi_busy) {
        return;                     // 上一帧还没传完, 这一拍跳过
    }

    rxdma_done = false;
    txdma_done = false;
    spi_busy = true;

    mt6835_cs(0);                   // 拉低 CS 选中芯片

    stat = hpm_spi_transmit_receive_nonblocking(MT6835_SPI, mt6835_tx_buf, mt6835_rx_buf, MT6835_FRAME_LEN);

    if (stat != status_success) {
        mt6835_cs(1);               // 启动失败要立刻放开 CS, 否则后面一直卡在 busy
        spi_busy = false;
        MT6835.error_cnt++;
    }
}

// 处理上一帧收到的数据, 更新机械角/电角度/电角速度
void mt6835_update_angle(void)
{
    static float prev_elec_angle;
    static uint32_t tick_cnt;
    static bool speed_ready;
    float delta;
    uint32_t raw;

    tick_cnt++;

    if (!frame_ready) {
        return;                     // 这一拍没收到新帧, 保持上一次的角度
    }
    frame_ready = false;

    // 3 字节拼成 24 位; MT6835 的 21 位角度在高 21 位, 低 3 位要丢掉
    raw = ((uint32_t)mt6835_rx_buf[2] << 16)
        | ((uint32_t)mt6835_rx_buf[3] << 8)
        | ((uint32_t)mt6835_rx_buf[4] & 0xF8U);

    // 减掉机械零位, 这一步就是常说的"角度对齐", 偏移量要实测标定
    MT6835.angle_raw = (int32_t)raw - MT6835_ANGLE_OFFSET;
    // 先掩到 24 位再换算: 直接 (uint32_t)angle_raw 的话, 负角度会变成巨大的正数(算出 1603 rad 那种)
    MT6835.angle_rad = (float)((uint32_t)MT6835.angle_raw & 0x00FFFFFFU) * (6.2831853f / 16777216.0f);   // 除以 2^24

    // 电角度 = 机械角 * 极对数, 取整圈
    MT6835.elec_angle = (float)(((uint32_t)MT6835.angle_raw * MT6835_POLE_PAIRS) & 0x00FFFFFFU)
                      * (6.2831853f / 16777216.0f);

    // 电角速度: 用两次的电角度差除以时间; 差值要归一化到 [-pi, pi], 否则过零点会跳变
    if (speed_ready && (tick_cnt > 0U)) {
        delta = MT6835.elec_angle - prev_elec_angle;
        if (delta > 3.14159265f) {
            delta -= 6.2831853f;
        } else if (delta < -3.14159265f) {
            delta += 6.2831853f;
        }
        MT6835.elec_speed = delta * ((float)MT6835_UPDATE_FREQ / (float)tick_cnt);
        MT6835.elec_speed_lpf += MT6835_SPEED_LPF_COEFF * (MT6835.elec_speed - MT6835.elec_speed_lpf);
    } else {
        MT6835.elec_speed = 0.0f;
        MT6835.elec_speed_lpf = 0.0f;
        speed_ready = true;
    }

    prev_elec_angle = MT6835.elec_angle;
    tick_cnt = 0U;
}
