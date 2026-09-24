# ADC 采样修复实施计划

> 本计划已实施完成（提交 `2dcab1b`），保留作记录。实现细节与实测硬件行为见
> `docs/superpowers/specs/2026-09-21-adc-sampling-repair-design.md`。

**目标：** 在不改变 PWM 采样点的前提下，让 ADC0/ADC1 的采样与 DMA 失败处理确定化。

**架构：** ADC0 作为 PWM 同步电流通道；ADC1 作为软件触发的电压 / 母线电流通道。
每个 DMA 结果都当作必须通过元数据校验的帧。

**技术栈：** HPM5E00 ADC16 驱动、PWMV2/TRGM、FreeRTOS C 应用、RISC-V GCC。

---

### Task 1：加固 ADC 接口与内存放置

**文件：** `adc.h` / `adc.c` / `foc.h`

- [x] `adc1_read_all` 返回 `bool`。
- [x] ADC1 使用 `ADC16_CONFIG_TRG0B`；两块 DMA 缓冲区改用非缓存对齐 BSS，并按抢占模式
      固定 192 字节（48 项）分配。
- [x] 增加 ADC1 读失败 / ADC0 坏帧 / ADC1 IBUS 失败计数。
- [x] 配置 PF27 / PF26 / PF21 / PF20 为模拟输入。

### Task 2：让 ADC1 读取事务化

**文件：** `adc.c` / `foc.c`

- [x] 写入输出前校验完成情况与四个通道的元数据。
- [x] 软件触发或超时失败时返回 `false`。
- [x] 只把成功读取的样本累加进中点。
- [x] 电压读取失败时跳过更新。

### Task 3：校验 ADC0 帧与配置

**文件：** `adc.c` / `foc.c` / `foc_task.c`

- [x] 保持 ADC1 `sel_sync_ahb = true`（ADC 控制器时钟与 DMA 总线时钟同为 AHB0；
      `/4` 只是转换时钟分频，与同步位无关）。
- [x] 拒绝通道 / 触发 / 序号元数据不符的 ADC0 DMA 帧。
- [x] 新增 ADC1 `TRG0C` 的 IBUS 低频采样队列，恢复 IBUS 中点 / 滤波 / 上报。
- [x] `adc1_read_ibus` 改用 DMA 元数据判完成（`TRG_CMPT_FLAG` 只对开 `INTEN` 的队列置位）。

### Task 4：构建与复查

- [x] 用缓存工具链编译。
- [x] 只修由本次改动引起的编译问题。
- [x] 确认没有调试插桩或无关文件被改。
