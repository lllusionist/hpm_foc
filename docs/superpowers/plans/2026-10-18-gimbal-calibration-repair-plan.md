# 云台电机校准修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the original open-loop voltage encoder alignment flow, stop immediately when offset quality is bad, and prevent invalid calibration data from reaching flux/J or Flash.

**Architecture:** Keep R, L, dir_pp, flux, and J as separate calibration states. The encoder alignment state becomes the only producer of the encoder offset: it first coasts to rest, ramps a voltage vector, performs slow CW/CCW voltage scans, computes offset/LUT quality, and either stops in IDLE or proceeds to flux. Flux no longer acts as an offset fallback.

**Tech Stack:** C firmware, HPM SDK/FreeRTOS, `foc_cal.c` calibration state machine, existing `FOC_voltage()` open-loop PWM path, existing PowerShell syntax-check command.

---

### Task 1: Remove Competing Offset Paths

**Files:**
- Modify: `foc/core/outer/src/foc_cal.c:109-121,457-620`

- [ ] **Step 1: Remove flux offset accumulation state**

Delete the `flux_enc_sin`, `flux_enc_cos`, `flux_enc_vel_sum`, and `flux_enc_cnt` statics and remove their reset in `CS_FLUX_START`.

- [ ] **Step 2: Remove flux offset sampling**

Remove the `enc_err_accum(FOC.calib_angle)` and `flux_enc_vel_sum += encoder.phase_vel` calls from `CS_FLUX_MEAS1/CS_FLUX_MEAS2`.

- [ ] **Step 3: Remove flux offset fallback**

Delete the `CS_FLUX_END` block that computes `delta`, writes `Config.encoder_offset`, clears `Config.offset_lut`, and prints `enc align(flux)`.

- [ ] **Step 4: Run syntax verification**

Run the repository's existing `foc_cal.c` syntax-check command from `D:\Desktop\FOC\foc\hpm5e00evk_flash_xip_debug`, using `compile_commands.json`, replacing `-c` with `-fsyntax-only`, and adding the two FreeRTOS include paths.

Expected: exit code `0`; the existing `portasmHAS_MTIME` warning may remain.

### Task 2: Restore Voltage-Mode Encoder Alignment

**Files:**
- Modify: `foc/core/outer/src/foc_cal.c:62-70,694-884`
- Modify: `foc/core/outer/inc/foc_cal.h:37-43`

- [ ] **Step 1: Keep only the required encoder states**

Retain `CS_ENCODER_START`, `CS_ENCODER_SETTLE`, `CS_ENCODER_RAMP`, `CS_ENCODER_CW_LOOP`, `CS_ENCODER_CCW_LOOP`, and `CS_ENCODER_END`. Do not add another current-loop alignment state.

- [ ] **Step 2: Define voltage alignment parameters**

Use these firmware constants and state variables:

```c
#define ENC_SLOW_DIV 2U
static float align_v;
static uint8_t enc_div_cnt;
```

Calculate `align_v` once in `CS_ENCODER_START`:

```c
float vlim = 0.8f * FOC.Vbus_filt * 0.57735f;
float imax = (Config.phase_resistance > 0.1f)
           ? (vlim / Config.phase_resistance) : Config.current_limit;
if (imax > 0.6f) imax = 0.6f;
if (imax < 0.3f) imax = 0.3f;
align_v = Config.phase_resistance * imax;
if (align_v < 0.2f) align_v = 0.2f;
if (align_v > Config.calib_voltage * 0.9f)
    align_v = Config.calib_voltage * 0.9f;
```

- [ ] **Step 3: Add stop and voltage ramp**

`CS_ENCODER_START` disables calibration current mode, turns PWM low sides off, resets `phase_set`, `sample_count`, `enc_runin`, and `enc_div_cnt`, then enters `CS_ENCODER_SETTLE`.

`CS_ENCODER_SETTLE` keeps `FOC.calib_mode=false` and waits for `fabsf(encoder.vel) < 0.05f` or a 2-second timeout, then resets `loop_count` and enters `CS_ENCODER_RAMP`.

`CS_ENCODER_RAMP` applies only the original open-loop voltage path:

```c
float ramp = time / 0.3f;
if (ramp > 1.0f) ramp = 1.0f;
FOC_voltage(align_v * ramp, 0.0f, 0.0f);
```

Hold the final voltage until `time > 0.8f`, then reset the scan counters and enter `CS_ENCODER_CW_LOOP`.

- [ ] **Step 4: Use voltage drag for both scan directions**

On every scan tick, hold the current voltage vector with:

```c
FOC_voltage(align_v, 0.0f, phase_set);
```

Advance `phase_set` every `ENC_SLOW_DIV` calibration-loop calls. Keep the existing one-revolution run-in, CW sampling, CCW sampling, and CW/CCW error averaging. Do not call `calib_drag_set()` or `calib_align_set()` from the encoder alignment states.

- [ ] **Step 5: Run syntax verification**

Run the same `foc_cal.c` syntax-check command.

Expected: exit code `0`.

### Task 3: Make Offset Quality the Flow Gate

**Files:**
- Modify: `foc/core/outer/src/foc_cal.c:808-884,1035-1088`

- [ ] **Step 1: Compute CW/CCW offset and quality once**

At the first `CS_ENCODER_END` pass (`sample_count == 0`), compute:

```c
Config.encoder_offset = mean(encoder_error_arr);
std_elec = stddev(encoder_error_arr) * pole_pairs * M_2PI / ENCODER_CPR_F;
enc_offset_ok = (std_elec < 0.785f);
```

Print exactly one quality line:

```text
enc align(drag): offset=... std=...deg -> ok/BAD
```

- [ ] **Step 2: Keep LUT generation only after offset quality passes**

If `enc_offset_ok` is false, clear `Config.offset_lut`, print the failure, disable calibration mode, stop PWM, set `Config.calib_valid=false`, set `mCalibStep=CS_NULL`, request `IDLE`, and `break` before building a LUT or entering flux.

If it is true, retain the existing LUT construction and pkpk check. If LUT pkpk exceeds `ENCODER_CPR/(2*pole_pairs)`, clear only the LUT and retain the validated offset.

- [ ] **Step 3: Remove offset fallback from final validity**

At the report stage, compute validity from the already validated offset and successful flux/J results. Do not recompute or replace the offset from flux data.

- [ ] **Step 4: Save only valid results**

Keep:

```c
if (Config.calib_valid) {
    CONFIG_save_to_flash();
} else {
    printf("calib: invalid -> NOT saved to flash\\n");
}
```

Do not call `CONFIG_load_from_flash()` during the running calibration task.

- [ ] **Step 5: Run syntax verification**

Run the same `foc_cal.c` syntax-check command.

Expected: exit code `0`.

### Task 4: Keep J Downstream of a Valid Offset

**Files:**
- Modify: `foc/core/outer/src/foc_cal.c:886-1040`

- [ ] **Step 1: Preserve the existing gentle J ramp**

Keep the current J settle, ramped open-loop spinup, ACC, and DEC implementation. It may only be reached after `CS_ENCODER_END` reports a valid offset.

- [ ] **Step 2: Remove offset validation from J**

J may report `a+`, `a-`, and `J` for informational speed-loop tuning, but it must not be used to rescue or validate a failed encoder offset.

- [ ] **Step 3: Verify no bad-path transition remains**

Search `foc_cal.c` and confirm every encoder-quality failure transitions directly to `CS_NULL`/`IDLE`, and the only normal transition after `CS_ENCODER_END` quality success is `CS_FLUX_START`.

- [ ] **Step 4: Run syntax verification**

Run the same `foc_cal.c` syntax-check command.

Expected: exit code `0`.

### Task 5: Review and Regression Verification

**Files:**
- Review: `foc/core/outer/src/foc_cal.c`
- Review: `foc/core/outer/inc/foc_cal.h`
- Review only: user-modified `foc/core/outer/inc/foc.h`, `foc/core/outer/src/config.c`, `foc/core/outer/src/foc.c`, and `foc/core/outer/src/vofa.c`

- [ ] **Step 1: Inspect the final diff**

Confirm the diff only contains the scheme-A calibration changes and does not alter the user-owned files listed above.

- [ ] **Step 2: Search for competing calibration drivers**

Confirm the encoder CW/CCW states call `FOC_voltage()` and do not call `calib_drag_set()` or `calib_align_set()`.

- [ ] **Step 3: Verify the failure path statically**

Confirm an offset-quality `BAD` path cannot reach `CS_FLUX_START`, `CS_MOTOR_J_START`, `CS_MOTOR_J_SPINUP`, or `CONFIG_save_to_flash()`.

- [ ] **Step 4: Run the final syntax verification**

Run the same syntax-check command after all edits.

Expected: exit code `0`.

- [ ] **Step 5: Hardware acceptance check**

After flashing, run calibration once and collect the serial log and waveform. Acceptance requires:

```text
enc align(drag): ... -> ok
```

Then flux and J may run. If the line reports `BAD`, calibration must stop before flux/J, report `valid=0`, and leave Flash unchanged.
