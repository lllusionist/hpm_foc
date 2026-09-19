#include <string.h>
#include "config.h"
#include "util.h"

tConfig Config;

void config_init(void)
{
    memset(&Config, 0, sizeof(tConfig));
    
    Config.magic = CONFIG_MAGIC;
    Config.calib_valid = false;
    
    Config.invert_motor_dir = 0;
    Config.pole_pairs      = 0;
    Config.current_limit   = 8.0f;
    Config.velocity_limit  = 30.0f;
    Config.calib_current   = 0.3f;
    Config.calib_voltage   = 3.0f;
    
    Config.default_op_mode        = CONTROL_MODE_POSITION_FILTER;
    Config.pos_p_gain             = 30.0f;
    Config.vel_p_gain             = 3.0f;
    Config.vel_i_gain             = 10.0f;
    Config.current_ff_gain        = 0.001f;
    Config.current_ctrl_bw        = 1000;
    Config.anticogging_enable     = 1;
    Config.target_velcity_window  = 0.5f;
    Config.target_position_window = 0.01f;
    Config.current_ramp_rate      = 25.0f;
    Config.velocity_ramp_rate     = 50.0f;
    Config.position_filter_bw     = 10;
    Config.profile_velocity       = 50;
    Config.profile_accel          = 50;
    Config.profile_decel          = 50;
    
    Config.protect_under_voltage = 22.0f;
    Config.protect_over_voltage  = 26.0f;
    Config.protect_over_current  = 20.0f;
    Config.protect_i_bus_max     = 20.0f;
    
    Config.node_id = 1;
    
    Config.cogging_map_crc = 0;
    Config.cogging_map_valid = 0;
    
    Config.crc32 = util_crc32((const uint8_t *)&Config, sizeof(Config));
}