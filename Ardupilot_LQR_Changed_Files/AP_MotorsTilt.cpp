/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_MotorsTilt.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsTilt::var_info[] = {

    // @Param: BAT_CURR_MAX
    // @DisplayName: Motor Current Max
    // @Description: Maximum current over which maximum throttle is limited (0 = Disabled)
    // @Range: 0 200
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("BAT_CURR_MAX", 12, AP_MotorsTilt, _batt_current_max, AP_MOTORS_BAT_CURR_MAX_DEFAULT),

    // 13, 14 were used by THR_MIX_MIN, THR_MIX_MAX

    // @Param: PWM_TYPE
    // @DisplayName: Output PWM type
    // @Description: This selects the output PWM type, allowing for normal PWM continuous output, OneShot, brushed or DShot motor output
    // @Values: 0:Normal,1:OneShot,2:OneShot125,3:Brushed,4:DShot150,5:DShot300,6:DShot600,7:DShot1200,8:PWMRange
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 15, AP_MotorsTilt, _pwm_type, PWM_TYPE_NORMAL),

    // @Param: PWM_MIN
    // @DisplayName: PWM output minimum
    // @Description: This sets the min PWM output value in microseconds that will ever be output to the motors
    // @Units: PWM
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("PWM_MIN", 16, AP_MotorsTilt, _pwm_min, 1000),

    // @Param: PWM_MAX
    // @DisplayName: PWM output maximum
    // @Description: This sets the max PWM value in microseconds that will ever be output to the motors
    // @Units: PWM
    // @Range: 0 2000
    // @User: Advanced
    AP_GROUPINFO("PWM_MAX", 17, AP_MotorsTilt, _pwm_max, 2000),

    // @Param: SPIN_ARM
    // @DisplayName: Motor Spin armed
    // @Description: Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range.  Should be lower than MOT_SPIN_MIN.
    // @Values: 0.0:Low, 0.1:Default, 0.2:High
    // @User: Advanced
    AP_GROUPINFO("SPIN_ARM", 19, AP_MotorsTilt, _spin_arm, AP_MOTORS_SPIN_ARM_DEFAULT),

    // @Param: THST_HOVER
    // @DisplayName: Thrust Hover Value
    // @Description: Motor thrust needed to hover expressed as a number from 0 to 1
    // @Range: 0.2 0.8
    // @User: Advanced
    AP_GROUPINFO("THST_HOVER", 21, AP_MotorsTilt, _throttle_hover, AP_MOTORS_THST_HOVER_DEFAULT),

    // @Param: YAW_SV_ANGLE
    // @DisplayName: Yaw Servo Max Lean Angle
    // @Description: Yaw servo's maximum lean angle (Tricopter only)
    // @Range: 5 80
    // @Units: deg
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO_FRAME("SV_ANGLE", 35, AP_MotorsTilt, _servo_angle_max_deg, 30, AP_PARAM_FRAME_TRICOPTER),

    // @Param: SPOOL_TIME
    // @DisplayName: Spool up time
    // @Description: Time in seconds to spool up the motors from zero to min throttle. 
    // @Range: 0.05 2
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SPOOL_TIME", 36, AP_MotorsTilt, _spool_up_time, AP_MOTORS_SPOOL_UP_TIME_DEFAULT),

    // @Param: SLEW_UP_TIME
    // @DisplayName: Output slew time for increasing throttle
    // @Description: Time in seconds to slew output from zero to full. This is used to limit the rate at which output can change. Range is constrained between 0 and 0.5.
    // @Range: 0 .5
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SLEW_UP_TIME", 40, AP_MotorsTilt, _slew_up_time, AP_MOTORS_SLEW_TIME_DEFAULT),

    // @Param: SLEW_DN_TIME
    // @DisplayName: Output slew time for decreasing throttle
    // @Description: Time in seconds to slew output from full to zero. This is used to limit the rate at which output can change.  Range is constrained between 0 and 0.5.
    // @Range: 0 .5
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SLEW_DN_TIME", 41, AP_MotorsTilt, _slew_dn_time, AP_MOTORS_SLEW_TIME_DEFAULT),

    // @Param: SAFE_TIME
    // @DisplayName: Time taken to disable and enable the motor PWM output when disarmed and armed.
    // @Description: Time taken to disable and enable the motor PWM output when disarmed and armed.
    // @Range: 0 5
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SAFE_TIME", 42, AP_MotorsTilt, _safe_time, AP_MOTORS_SAFE_TIME_DEFAULT),

    // @Param: SPOOL_TIM_DN
    // @DisplayName: Spool down time
    // @Description: Time taken to spool down the motors from min to zero throttle. If set to 0 then SPOOL_TIME is used instead.
    // @Range: 0 2
    // @Units: s
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("SPOOL_TIM_DN", 44, AP_MotorsTilt, _spool_down_time, 0),

    AP_GROUPEND
};

// Constructor
AP_MotorsTilt::AP_MotorsTilt(uint16_t speed_hz) :
                AP_MotorsMulticopter(speed_hz),
                _throttle_limit(1.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
};

void AP_MotorsTilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);
    add_motor_num(AP_MOTORS_MOT_4);
    
    add_motor_num(AP_MOTORS_MOT_5);
    add_motor_num(AP_MOTORS_MOT_6);
    add_motor_num(AP_MOTORS_MOT_7);
    add_motor_num(AP_MOTORS_MOT_8);

    add_motor_num(AP_MOTORS_MOT_9);
    add_motor_num(AP_MOTORS_MOT_10);
    add_motor_num(AP_MOTORS_MOT_11);
    add_motor_num(AP_MOTORS_MOT_12);

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    _mav_type = MAV_TYPE_GENERIC;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(true);
}


// output - sends commands to the motors
void AP_MotorsTilt::output()
{
    // run spool logic
    output_logic();

    // convert rpy_thrust values to pwm
    output_to_motors();

};


uint16_t AP_MotorsTilt::omega2pwm(double omega) {
    float pwm = std::round(omega/AP_MOTORS_TILT_MAX_OMEGA * 1000 + 1000);
    if (pwm > 2000) pwm = 2000;
    if (pwm < 1000) pwm = 1000;
    return static_cast<uint16_t>(pwm);
}

int16_t AP_MotorsTilt::angle2pwmProx(double angle) {

    float pwm = std::round(0.8 * angle/AP_MOTORS_TILT_MAX_ANGLE * 1000 + 1500);
    if (pwm > 2200) pwm = 2200;
    if (pwm < 800) pwm = 800;
    return static_cast<int16_t>(pwm);
}

int16_t AP_MotorsTilt::angle2pwmDist(double angle) {

    float pwm = std::round(1.275 * -angle/AP_MOTORS_TILT_MAX_ANGLE * 1000 + 1500);
    if (pwm > 2200) pwm = 2200;
    if (pwm < 800) pwm = 800;
    return static_cast<int16_t>(pwm);
}

void AP_MotorsTilt::output_to_motors()
{
    
    //for (int i = 0; i<12; i++)
    //{
        //if (i<4) ::printf("Actuator (%d) PWM = %d\n", i, omega2pwm(_actuator[i]));
        //else ::printf("Actuator (%d) PWM = %d\n", i, angle2pwm(_actuator[i]));
    //}

    rc_write(AP_MOTORS_MOT_1, omega2pwm(_actuator[0]));
    rc_write(AP_MOTORS_MOT_2, omega2pwm(_actuator[3]));
    rc_write(AP_MOTORS_MOT_3, omega2pwm(_actuator[6]));
    rc_write(AP_MOTORS_MOT_4, omega2pwm(_actuator[9]));

    rc_write(AP_MOTORS_MOT_5, angle2pwmProx(_actuator[1]));
    rc_write(AP_MOTORS_MOT_6, angle2pwmProx(_actuator[4]));
    rc_write(AP_MOTORS_MOT_7, angle2pwmProx(_actuator[7]));
    rc_write(AP_MOTORS_MOT_8, angle2pwmProx(_actuator[10]));

    rc_write(AP_MOTORS_MOT_9,  angle2pwmDist(_actuator[2]));
    rc_write(AP_MOTORS_MOT_10, angle2pwmDist(_actuator[5]));
    rc_write(AP_MOTORS_MOT_11, angle2pwmDist(_actuator[8]));
    rc_write(AP_MOTORS_MOT_12, angle2pwmDist(_actuator[11]));
}

// sends minimum values out to the motors
void AP_MotorsTilt::output_min()
{
    
    rc_write(AP_MOTORS_MOT_1, 900);
    rc_write(AP_MOTORS_MOT_2, 900);
    rc_write(AP_MOTORS_MOT_3, 900);
    rc_write(AP_MOTORS_MOT_4, 900);

    rc_write(AP_MOTORS_MOT_5, angle2pwmProx(0));
    rc_write(AP_MOTORS_MOT_6, angle2pwmProx(0));
    rc_write(AP_MOTORS_MOT_7, angle2pwmProx(0));
    rc_write(AP_MOTORS_MOT_8, angle2pwmProx(0));

    rc_write(AP_MOTORS_MOT_9, angle2pwmDist(0));
    rc_write(AP_MOTORS_MOT_10, angle2pwmDist(0));
    rc_write(AP_MOTORS_MOT_11, angle2pwmDist(0));
    rc_write(AP_MOTORS_MOT_12, angle2pwmDist(0));
}

// update the throttle input filter
void AP_MotorsTilt::update_throttle_filter()
{
    //maybe implement low pass for all actuators
}

// 10hz logging of voltage scaling and max trust
// void AP_MotorsTilt::Log_Write()
// {
//     const struct log_MotBatt pkt_mot {
//         LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
//         time_us         : AP_HAL::micros64(),
//         th_limit        : _throttle_limit,
//         th_average_max  : _throttle_avg_max,
//         th_out          : _throttle_out,
//         mot_fail_flags  : (uint8_t)(_thrust_boost | (_thrust_balanced << 1U)),
//     };
//     AP::logger().WriteBlock(&pkt_mot, sizeof(pkt_mot));
// }


// run spool logic
void AP_MotorsTilt::output_logic()
{
    const constexpr float minimum_spool_time = 0.05f;
    if (armed()) {
        if (_disarm_disable_pwm && (_disarm_safe_timer < _safe_time)) {
            _disarm_safe_timer += _dt;
        } else {
            _disarm_safe_timer = _safe_time;
        }
    } else {
           _disarm_safe_timer = 0.0f;
    }

    // force desired and current spool mode if disarmed or not interlocked
    if (!armed() || !get_interlock()) {
        _spool_desired = DesiredSpoolState::SHUT_DOWN;
        _spool_state = SpoolState::SHUT_DOWN;
    }

    if (_spool_up_time < minimum_spool_time) {
        // prevent float exception
        _spool_up_time.set(minimum_spool_time);
    }

    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        // Motors should be stationary.
        // Servos set to their trim values or in a test condition.

        // set limits flags
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;
        limit.throttle_upper = true;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired != DesiredSpoolState::SHUT_DOWN && _disarm_safe_timer >= _safe_time.get()) {
            _spool_state = SpoolState::GROUND_IDLE;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 0.0f;
        _throttle_thrust_max = 0.0f;

        // initialise motor failure variables
        _thrust_boost = false;
        _thrust_boost_ratio = 0.0f;
        break;

    case SpoolState::GROUND_IDLE: {
        // Motors should be stationary or at ground idle.
        // Servos should be moving to correct the current attitude.

        // set limits flags
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
        limit.throttle_lower = true;
        limit.throttle_upper = true;

        // set and increment ramp variables
        switch (_spool_desired) {
        case DesiredSpoolState::SHUT_DOWN: {
            const float spool_time = _spool_down_time > minimum_spool_time ? _spool_down_time : _spool_up_time;
            const float spool_step = _dt / spool_time;
            _spin_up_ratio -= spool_step;
            // constrain ramp value and update mode
            if (_spin_up_ratio <= 0.0f) {
                _spin_up_ratio = 0.0f;
                _spool_state = SpoolState::SHUT_DOWN;
            }
            break;
        }

        case DesiredSpoolState::THROTTLE_UNLIMITED: {
            const float spool_step = _dt / _spool_up_time;
            _spin_up_ratio += spool_step;
            // constrain ramp value and update mode
            if (_spin_up_ratio >= 1.0f) {
                _spin_up_ratio = 1.0f;
                if (!get_spoolup_block()) {
                    // Only advance from ground idle if spoolup checks have passed
                    _spool_state = SpoolState::SPOOLING_UP;
                }
            }
            break;
        }
        case DesiredSpoolState::GROUND_IDLE: {
            const float spool_up_step = _dt / _spool_up_time;
            const float spool_down_time = _spool_down_time > minimum_spool_time ? _spool_down_time : _spool_up_time;
            const float spool_down_step = _dt / spool_down_time;
            float spin_up_armed_ratio = 0.0f;
            _spin_up_ratio += constrain_float(spin_up_armed_ratio - _spin_up_ratio, -spool_down_step, spool_up_step);
            break;
        }
        }
        _throttle_thrust_max = 0.0f;

        // initialise motor failure variables
        _thrust_boost = false;
        _thrust_boost_ratio = 0.0f;
        break;
    }
    case SpoolState::SPOOLING_UP: {
        const float spool_step = _dt / _spool_up_time;
        // Maximum throttle should move from minimum to maximum.
        // Servos should exhibit normal flight behavior.

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED) {
            _spool_state = SpoolState::SPOOLING_DOWN;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 1.0f;
        _throttle_thrust_max += spool_step;

        // initialise motor failure variables
        _thrust_boost = false;
        _thrust_boost_ratio = MAX(0.0, _thrust_boost_ratio - spool_step);
        break;
    }

    case SpoolState::THROTTLE_UNLIMITED: {
        const float spool_step = _dt / _spool_up_time;
        // Throttle should exhibit normal flight behavior.
        // Servos should exhibit normal flight behavior.

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired != DesiredSpoolState::THROTTLE_UNLIMITED) {
            _spool_state = SpoolState::SPOOLING_DOWN;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 1.0f;

        if (_thrust_boost && !_thrust_balanced) {
            _thrust_boost_ratio = MIN(1.0, _thrust_boost_ratio + spool_step);
        } else {
            _thrust_boost_ratio = MAX(0.0, _thrust_boost_ratio - spool_step);
        }
        break;
    }

    case SpoolState::SPOOLING_DOWN:
        // Maximum throttle should move from maximum to minimum.
        // Servos should exhibit normal flight behavior.

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // make sure the motors are spooling in the correct direction
        if (_spool_desired == DesiredSpoolState::THROTTLE_UNLIMITED) {
            _spool_state = SpoolState::SPOOLING_UP;
            break;
        }

        // set and increment ramp variables
        _spin_up_ratio = 1.0f;
        const float spool_time = _spool_down_time > minimum_spool_time ? _spool_down_time : _spool_up_time;
        const float spool_step = _dt / spool_time;
        _throttle_thrust_max -= spool_step;

        // constrain ramp value and update mode
        if (_throttle_thrust_max <= 0.0f) {
            _throttle_thrust_max = 0.0f;
        }
        _thrust_boost_ratio = MAX(0.0, _thrust_boost_ratio - spool_step);
        break;
    }
}

// passes throttle directly to all motors for ESC calibration.
//   throttle_input is in the range of 0 ~ 1 where 0 will send get_pwm_output_min() and 1 will send get_pwm_output_max()
void AP_MotorsTilt::set_throttle_passthrough_for_esc_calibration(float throttle_input)
{
    if (armed()) {
        uint16_t pwm_out = get_pwm_output_min() + constrain_float(throttle_input, 0.0f, 1.0f) * (get_pwm_output_max() - get_pwm_output_min());
        // send the pilot's input directly to each enabled motor
        for (uint16_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rc_write(i, pwm_out);
            }
        }
    }
}

// output a thrust to all motors that match a given motor mask. This
// is used to control tiltrotor motors in forward flight. Thrust is in
// the range 0 to 1
void AP_MotorsTilt::output_motor_mask(float thrust, uint16_t mask, float rudder_dt)
{
    const int16_t pwm_min = get_pwm_output_min();
    const int16_t pwm_range = get_pwm_output_max() - pwm_min;

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if ((mask & (1U << i)) && armed() && get_interlock()) {
                /*
                 apply rudder mixing differential thrust
                 copter frame roll is plane frame yaw as this only
                 apples to either tilted motors or tailsitters
                 */
                int16_t pwm_output = pwm_min + pwm_range * _actuator[i];
                rc_write(i, pwm_output);
            } else {
                rc_write(i, pwm_min);
            }
        }
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsTilt::get_motor_mask()
{
    return SRV_Channels::get_output_channel_mask(SRV_Channel::k_boost_throttle);
}

// convert to PWM min and max in the motor lib
void AP_MotorsTilt::convert_pwm_min_max_param(int16_t radio_min, int16_t radio_max)
{
    if (_pwm_min.configured() || _pwm_max.configured()) {
        return;
    }
    _pwm_min.set_and_save(radio_min);
    _pwm_max.set_and_save(radio_max);
}

bool AP_MotorsTilt::arming_checks(size_t buflen, char *buffer) const
{
    // run base class checks
    if (!AP_Motors::arming_checks(buflen, buffer)) {
        return false;
    }

    // Check output function is setup for each motor
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!motor_enabled[i]) {
            continue;
        }
        uint8_t chan;
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);
        if (!SRV_Channels::find_channel(function, chan)) {
            hal.util->snprintf(buffer, buflen, "no SERVOx_FUNCTION set to Motor%u", i + 1);
            return false;
        }
    }


    return true;
}

// set update rate to motors - a value in hertz
void AP_MotorsTilt::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
	    1U << AP_MOTORS_MOT_4;
    rc_set_freq(mask, _speed_hz);
}

