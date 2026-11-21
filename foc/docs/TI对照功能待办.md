# 有感 FOC 功能待办

2026-11-22 整理。对照当前 `dev_v1.1` 有感固件与本地 TI `ENC` 示例。文中 TI 源码路径相对于 `D:\Desktop\ti_foc_split\ENC`。

## 优先实现

### ADC 零点自检

- [ ] 在 `foc_adc_midpoint()` 采够有效帧后，检查三相和母线电流的零点均值是否处于板上采样电路的合理范围，通过后再置 `adc_ready`。阈值按实际偏置电路和正常读数确定。
- [ ] 零点异常时保持驱动失能，报告异常通道和原始中点。覆盖有效帧不足、零点越界和自检通过三种情况。

TI 在 `common/source/motor1_drive.c:1060` 对电流偏置检查上下限，越界时置 `currentOffset` 故障。当前工程在取得足够有效样本后直接用均值作为零点。[当前零点计算](../core/outer/src/foc.c#L108) · [使能入口](../core/outer/src/foc.c#L299)

### 设备状态回读与停机历史

- [ ] 上位机接入 `0x62` 的 `$FOCSW` 回包，显示设备回读的 `ARM`、`ON`、`REACH` 和 `ERRORS`。使能请求发出后等待设备确认，通信断开时显示状态未知。
- [ ] 区分未校准拒绝使能、手动停机、保护关断和心跳超时。保存最近一次异常停机原因，使上位机重连后仍能读到。当前故障和历史原因各自显示。

TI 在 `common/source/motor_common.c:131` 将当前故障累积到 `faultMtrPrev`。当前固件已有状态字，心跳超时原因目前由 UART 打印。[状态字回包](../core/outer/src/vofa.c#L28) · [心跳关断](../core/outer/src/foc_task.c#L388)

## 运行诊断与保护

- [ ] **实测超速。** 基于编码器机械转速设置独立阈值和持续时间，满足条件后锁存故障并关断。当前 `velocity_limit` 用于约束速度或电流给定。
- [ ] **三相 RMS、缺相与不平衡。** 从 `foc_adc_callback()` 重构 `Ic` 前的实测三相电流统计有效值。先回传诊断值和告警，再根据有效采样、转动状态及持续时间决定是否关断。
- [ ] **堵转、启动失败与持续过载。** 结合运动指令、实测速度、电流或功率和持续时间。堵转与启动失败的判据只在控制器要求运动的阶段生效，位置保持使用相应的保持状态。
- [ ] **板温与热保护。** 读取预留的 NTC 通道，校对温度换算后回传板温，再根据板级能力设定降额或过温关断阈值。

TI 的 `common/source/motor_common.c:530` 包含扩展运行监控，`common/include/motor_common.h:247` 的默认关断掩码主要覆盖基础电气和偏置类故障。当前工程可复用 `foc_check_and_trip()` 的故障锁存与关断路径。[速度给定限制](../core/outer/src/foc_ctrl.c#L355) · [实测三相电流](../core/outer/src/foc.c#L455) · [运行保护入口](../core/outer/src/foc_task.c#L213) · [NTC 接口](项目描述.md#3-硬件资源)

## 电流模式切换到速度模式

- [ ] 为运行中的电流 → 速度切换增加接管路径。用切换时的实测转速初始化速度参考，结合前馈预置速度积分，让首拍 `Iq_set` 接近切换前的给定。
- [ ] 核对速度参考后续按原斜坡跟踪目标，且切换前后的电流给定受同一上限约束。针对接管路径保留连续的内环状态。

TI 在 `common/source/motor1_drive.c:1770` 的手动转矩路径给速度 PI 预置积分。当前运行中切模式会调用 `CONTROLLER_reset()`，清零速度给定和内外环积分。[当前模式切换](../core/outer/src/foc_ctrl.c#L100)

## 按应用需求扩展

- [ ] **硬件异步过流关断。** 参考 TI `f28003x/drivers/source/hal.c:807` 的比较器到 PWM Trip Zone 路径，先核实本板的故障信号源和外设路由。当前板的相过流关断在 ADC 中断执行。[板级限制](限制.md#1-硬件限制)
- [ ] **弱磁、MTPA 与过调制。** 高速运行出现电压饱和时评估弱磁。根据目标电机的凸极特性决定是否做 MTPA。过调制结合功率级和相电流采样窗口评估。
- [ ] **SFRA 扫频。** 需要测环路幅相或稳定裕度时，增加小信号注入和响应采集。已有的 `Iq` 阶跃用于时域响应观察。[电流阶跃采集](../core/outer/src/foc.c#L529)

弱磁、MTPA、过调制和 SFRA 在这份 TI `ENC` 工程的 `f28003x/ccs/motor_control/universal_motorcontrol_lab.projectspec` 中默认关闭。

## 工程配套

- [ ] **录波追溯。** CSV 保存固件版本、设备回读参数，并记录采样序号或丢帧信息，供回放时核对时间轴。[录波元数据](../../tools/foc_scope/focscope/main_window.py#L808) · [接收丢帧计数](../../tools/foc_scope/focscope/serial_io.py#L129)
- [ ] **构建与回归。** 固定 SEGGER Debug 的构建入口，将已有固件和上位机测试串到同一命令，补齐 CSV 回放所需的 NumPy 依赖。[固件测试](../tests) · [上位机依赖](../../tools/foc_scope/requirements.txt)
