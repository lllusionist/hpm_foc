# 云台电机校准修复设计

## 目标

让校准程序在换用云台电机后仍能完整、可诊断地运行，重点修复编码器 offset 对齐阶段的毛刺、失步和错误 offset 进入 J 阶段的问题。

## 根因

原程序的编码器拖动使用开环电压 `FOC_voltage(drag_v, 0, phase_set)`。后续修改把该阶段改成了电流闭环拖动。云台电机存在较大的齿槽/静摩擦，转子粘滑时电流环会追逐变化的反电势和角度，产生电压饱和与电流毛刺。

航模电机可以工作，是因为其转矩裕量足够，开环磁场能平滑牵引转子。云台电机必须保留开环电压拖动，并在进入拖动前停稳、缓升电压。

## 状态流转

```text
R -> L -> dir_pp -> encoder settle/ramp -> encoder CW -> encoder CCW
  -> encoder quality check
  -> BAD: stop, invalid, do not enter flux/J, do not save
  -> OK: flux -> J -> report/save
```

编码器 offset 对齐失败时，不再使用 flux offset 作为自动后备，也不继续执行 J。J 依赖绝对编码器角度，不能用于验证一个已经失败的 offset。

## 编码器对齐

- 使用原版开环电压拖动：`FOC_voltage(align_v, 0, phase_set)`。
- `align_v` 根据 `R * I` 计算，并受 `calib_voltage` 与母线电压限制。
- dir_pp 结束后先关闭 PWM，等待编码器速度接近零。
- 以 0.3s 电压斜坡吸合转子，再进入慢速 CW 扫描。
- CW 和 CCW 各扫描一圈，使用同一采样索引的误差平均抵消摩擦滞后。
- 扫描结束后计算 offset 误差标准差和 LUT 峰峰值。
- 标准差或 LUT 质量不合格时，直接停止校准。

## 失败处理

- offset 对齐失败：设置 `calib_valid=false`，关闭 PWM，进入 IDLE。
- 不运行 flux 和 J，避免产生误导性的磁链/J 数据。
- 不调用运行时 `CONFIG_load_from_flash()`，避免 XIP 环境下读 Flash 导致异常。
- 只有最终校准通过时才调用 `CONFIG_save_to_flash()`。

## J 阶段

J 只在 encoder offset 已通过质量检查后执行。J 使用编码器角度换相，保留温和的开环起转斜坡，然后进行 +Iq/-Iq 加减速斜率测量。J 只作为惯量和速度环参数建议，不再承担 offset 的验证职责。

## 验证标准

成功日志必须满足：

```text
enc align(drag): ... -> ok
J spinup: ... vel=...
J: a+ positive, a- negative, J positive
calib gate: ... -> valid=1
```

失败日志必须在 encoder 对齐阶段停止，并且不出现 flux/J 阶段输出，不写入 Flash。
