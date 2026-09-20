#include "foc_ctrl.h"
#include "config.h"
#include "encoder.h"
#include "foc_task.h"
#include <stdio.h>

tController Controller;

#define CTRL_PERIOD (1.0f / 1000.0f) // Task_FOC 周期 (1kHz)

void CONTROLLER_update_input_pos_filter_gain(float bw)
{
    float bandwidth = bw * M_2PI;
    Controller.input_pos_filter_ki = 2.0f * bandwidth;
    Controller.input_pos_filter_kp = 0.25f * SQ(Controller.input_pos_filter_ki);
}

void CONTROLLER_init(void)
{
    Controller.ctrl_mode      = (tControlMode)Config.default_op_mode;
    Controller.target_reached = false;

    Controller.op_mode_pending     = false;
    Controller.home_pending        = false;
    Controller.config_cmd_pending  = 0;

    CONTROLLER_update_input_pos_filter_gain(Config.position_filter_bw);
}

void CONTROLLER_reset(void)
{
    taskENTER_CRITICAL();

    float pos_meas = encoder.pos;

    Controller.input_position = pos_meas;
    Controller.input_velocity = 0.0f;
    Controller.input_current  = 0.0f;

    Controller.input_position_buffer = pos_meas;
    Controller.input_velocity_buffer = 0.0f;
    Controller.input_current_buffer  = 0.0f;

    Controller.pos_setpoint = pos_meas;
    Controller.vel_setpoint = 0.0f;
    Controller.cur_setpoint = 0.0f;

    Controller.vel_integrator = 0;

    Controller.input_updated  = false;
    Controller.target_reached = false;

    FOC.current_ctrl_integral_d = 0;
    FOC.current_ctrl_integral_q = 0;

    taskEXIT_CRITICAL();
}

int CONTROLLER_set_op_mode(tControlMode mode)
{
    Controller.ctrl_mode = mode;
    CONTROLLER_reset();

    return 0;
}

int CONTROLLER_set_home(void)
{
    int ret = 0;

    if (foc_get_stage() == IDLE) {
        taskENTER_CRITICAL();
        encoder.shadow_count = 0;
        taskEXIT_CRITICAL();
    } else if (foc_get_stage() == RUN) {
        float pos_err = Controller.pos_setpoint - encoder.pos;
        if ((ABS(encoder.vel) < 0.5f) && (ABS(pos_err) < 0.01f)) {
            taskENTER_CRITICAL();

            encoder.shadow_count = 0;
            encoder.pos          = 0;
            encoder.pll_pos      = (float)encoder.count_aligned; // PLL 对齐当前单圈位置
            encoder.pll_vel      = 0;
            encoder.count_in_cpr_prev = encoder.count_in_cpr; // 防止产生错误 delta

            Controller.input_position        = 0;
            Controller.input_position_buffer = 0;
            Controller.pos_setpoint          = 0;
            Controller.vel_setpoint          = 0;
            Controller.cur_setpoint          = 0;

            taskEXIT_CRITICAL();
        } else {
            ret = -1;
        }
    } else {
        ret = -1;
    }

    return ret;
}

/* 命令写入缓冲后立即锁存 (USB/CAN 调用); 状态不满足则只存缓冲 */
void CONTROLLER_sync_callback(void)
{
    if (foc_get_stage() != RUN) {
        return;
    }

    switch (Controller.ctrl_mode) {
    case CONTROL_MODE_CURRENT_RAMP:
        Controller.input_current = Controller.input_current_buffer;
        break;

    case CONTROL_MODE_VELOCITY_RAMP:
        Controller.input_velocity = Controller.input_velocity_buffer;
        break;

    case CONTROL_MODE_POSITION_FILTER:
    case CONTROL_MODE_POSITION_PROFILE:
        Controller.input_position = Controller.input_position_buffer;
        break;

    default:
        break;
    }

    Controller.target_reached = false;
}

/* 1kHz 外环: 生成 FOC.Iq_set, 电流环仍在 ADC 中断里执行 */
void CONTROLLER_loop(void)
{
    float iq_set = 0;
    float v_err = 0.0f;
    tFSMState stage = foc_get_stage();

    // 处理中断里提交的请求 (必须在任务上下文执行)
    if ((stage == IDLE) || (stage == RUN)) {
        if (Controller.op_mode_pending) {
            Controller.op_mode_pending = false;
            CONTROLLER_set_op_mode(Controller.op_mode_req);
        }
        if (Controller.home_pending) {
            Controller.home_pending = false;
            CONTROLLER_set_home();
        }
        if (Controller.config_cmd_pending != 0U) {
            uint8_t cmd = Controller.config_cmd_pending;
            Controller.config_cmd_pending = 0;
            if (FOC.is_armed) {
                printf("cfg cmd: motor armed, refused\n");
            } else if (cmd == 1U) {
                printf("cfg save: %s\n",
                       (CONFIG_save_to_flash() == 0) ? "ok" : "fail");
            } else if (cmd == 2U) {
                config_init();
                CONFIG_save_to_flash();
                printf("cfg reset\n");
            }
        }
    }

    if (stage != RUN) {
        return;
    }

    const float pos_meas       = encoder.pos;
    const float vel_meas       = encoder.vel; // rps
    float       vel_des        = 0.0f;

    switch (Controller.ctrl_mode) {
    case CONTROL_MODE_CURRENT_RAMP: {
        float max_step_size = ABS(CTRL_PERIOD * Config.current_ramp_rate);
        float full_step     = Controller.input_current - Controller.cur_setpoint;
        float step          = CLAMP(full_step, -max_step_size, max_step_size);
        Controller.cur_setpoint += step;
    } break;

    case CONTROL_MODE_VELOCITY_RAMP: {
        float max_step_size = ABS(CTRL_PERIOD * Config.velocity_ramp_rate);
        float full_step     = Controller.input_velocity - Controller.vel_setpoint;
        float step          = CLAMP(full_step, -max_step_size, max_step_size);
        Controller.vel_setpoint += step;
        Controller.cur_setpoint = Config.current_ff_gain * (step / CTRL_PERIOD);

        if (!Controller.target_reached) {
            if (ABS(Controller.input_velocity - vel_meas) < Config.target_velcity_window) {
                Controller.target_reached = true;
            }
        }
    } break;

    case CONTROL_MODE_POSITION_FILTER:
    case CONTROL_MODE_POSITION_PROFILE: {
        // 二阶位置跟踪滤波
        float delta_pos = Controller.input_position - Controller.pos_setpoint;
        float delta_vel = Controller.input_velocity - Controller.vel_setpoint;
        float accel     = Controller.input_pos_filter_kp * delta_pos +
                      Controller.input_pos_filter_ki * delta_vel;
        Controller.cur_setpoint = Config.current_ff_gain * accel;
        Controller.vel_setpoint += CTRL_PERIOD * accel;
        Controller.pos_setpoint += CTRL_PERIOD * Controller.vel_setpoint;

        if (!Controller.target_reached) {
            if (ABS(Controller.input_position - pos_meas) < Config.target_position_window) {
                Controller.target_reached = true;
            }
        }
    } break;

    default:
        break;
    }

    vel_des = Controller.vel_setpoint;

    // 位置环 P
    if (Controller.ctrl_mode >= CONTROL_MODE_POSITION_FILTER) {
        float pos_err = Controller.pos_setpoint - pos_meas;
        vel_des += Config.pos_p_gain * pos_err;
    }

    // 速度限幅
    vel_des = CLAMP(vel_des, -Config.velocity_limit, +Config.velocity_limit);

    // 速度环 PI
    iq_set = Controller.cur_setpoint;

    if (Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) {
        v_err = vel_des - vel_meas;
        iq_set += Config.vel_p_gain * v_err;
        iq_set += Controller.vel_integrator;
    }

    // 电流模式下的速度限幅
    if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP) {
        float Imax = (+Config.velocity_limit - vel_meas) * Config.vel_p_gain;
        float Imin = (-Config.velocity_limit - vel_meas) * Config.vel_p_gain;
        iq_set     = CLAMP(iq_set, Imin, Imax);
    }

    // 电流限幅
    bool limited = false;
    if (iq_set > +Config.current_limit) {
        limited = true;
        iq_set  = +Config.current_limit;
    }
    if (iq_set < -Config.current_limit) {
        limited = true;
        iq_set  = -Config.current_limit;
    }

    FOC.Id_set = 0.0f;
    FOC.Iq_set = iq_set;

    // 速度积分器 (抗饱和)
    if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP) {
        Controller.vel_integrator = 0.0f;
    } else {
        if (limited) {
            Controller.vel_integrator *= 0.99f;
        } else {
            Controller.vel_integrator +=
                (Config.vel_i_gain * CTRL_PERIOD) * v_err;
        }
    }
}
