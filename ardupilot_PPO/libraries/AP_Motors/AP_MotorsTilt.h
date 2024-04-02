/// @file	AP_MotorsMulticopter.h
/// @brief	Motor control class for Multicopters
#pragma once

#include "AP_MotorsMulticopter.h"
#include <AP_Math/vectorN.h>

#define AP_MOTORS_THST_EXPO_DEFAULT     0.65f   // set to 0 for linear and 1 for second order approximation
#define AP_MOTORS_THST_HOVER_DEFAULT    0.35f   // the estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_TC         10.0f   // time constant used to update estimated hover throttle, 0 ~ 1
#define AP_MOTORS_THST_HOVER_MIN        0.125f  // minimum possible hover throttle
#define AP_MOTORS_THST_HOVER_MAX        0.6875f // maximum possible hover throttle
#define AP_MOTORS_SPIN_MIN_DEFAULT      0.15f   // throttle out ratio which produces the minimum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_MAX_DEFAULT      0.95f   // throttle out ratio which produces the maximum thrust.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_SPIN_ARM_DEFAULT      0.10f   // throttle out ratio which produces the armed spin rate.  (i.e. 0 ~ 1 ) of the full throttle range
#define AP_MOTORS_BAT_VOLT_MAX_DEFAULT  0.0f    // voltage limiting max default
#define AP_MOTORS_BAT_VOLT_MIN_DEFAULT  0.0f    // voltage limiting min default (voltage dropping below this level will have no effect)
#define AP_MOTORS_BAT_CURR_MAX_DEFAULT  0.0f    // current limiting max default
#define AP_MOTORS_BAT_CURR_TC_DEFAULT   5.0f    // Time constant used to limit the maximum current
#define AP_MOTORS_SLEW_TIME_DEFAULT     0.0f    // slew rate limit for thrust output
#define AP_MOTORS_SAFE_TIME_DEFAULT     1.0f    // Time for the esc when transitioning between zero pwm to minimum
#define AP_MOTORS_TILT_MAX_ANGLE             1.57f
#define AP_MOTORS_TILT_MAX_OMEGA             633

// spool definition
#define AP_MOTORS_SPOOL_UP_TIME_DEFAULT 0.5f    // time (in seconds) for throttle to increase from zero to min throttle, and min throttle to full throttle.


/// @class      AP_MotorsMulticopter
class AP_MotorsTilt : public AP_MotorsMulticopter {
public:

    // Constructor
    AP_MotorsTilt(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // output - sends commands to the motors
    inline void         updateControl(const VectorN<float, 12> &LL_Control) {for(int i = 0; i<12; i++) _actuator[i] = LL_Control[i];}
    void                output_to_motors() override;
    virtual void        output() override;
    void                output_logic();

    uint16_t            omega2pwm(double omega);
    int16_t             angle2pwmProx(double angle);
    int16_t             angle2pwmDist(double angle);


    // output_min - sends minimum values out to the motors
    void                output_min() override;

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // get throttle required to hover
    virtual float       get_throttle_hover() const override { return constrain_float(_throttle_hover, AP_MOTORS_THST_HOVER_MIN, AP_MOTORS_THST_HOVER_MAX); }

    // passes throttle directly to all motors for ESC calibration.
    //   throttle_input is in the range of 0 ~ 1 where 0 will send get_pwm_output_min() and 1 will send get_pwm_output_max()
    void                set_throttle_passthrough_for_esc_calibration(float throttle_input);

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    virtual void        output_motor_mask(float thrust, uint16_t mask, float rudder_dt) override;

    // returns maximum thrust in the range 0 to 1
    float               get_throttle_thrust_max() const { return _throttle_thrust_max; }

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint32_t    get_motor_mask() override;

    // get minimum or maximum pwm value that can be output to motors
    int16_t             get_pwm_output_min() const { return _pwm_min; }
    int16_t             get_pwm_output_max() const { return _pwm_max; }

    // return whether a motor is enabled or not
    bool                is_motor_enabled(uint8_t i) override { return motor_enabled[i]; }

    // convert values to PWM min and max if not configured
    void                convert_pwm_min_max_param(int16_t radio_min, int16_t radio_max);

    // 10hz logging of voltage scaling and max trust
    // void                Log_Write() override;

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    const char* _get_frame_string() const override { return "TILT"; }

    void output_armed_stabilizing() override {}

    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override {rc_write(motor_seq, pwm);}

    void set_actuator(int idx, float actuator) {_actuator[idx] = actuator; };


    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // output_to_motors - sends commands to the motors
    // void        output_to_motors();

    // update the throttle input filter
    virtual void        update_throttle_filter() override;

    // parameters
    AP_Float            _slew_up_time;          // throttle increase slew limitting
    AP_Float            _slew_dn_time;          // throttle decrease slew limitting
    AP_Float            _safe_time;             // Time for the esc when transitioning between zero pwm to minimum
    AP_Float            _spin_arm;              // throttle out ratio which produces the armed spin rate.  (i.e. 0 ~ 1 ) of the full throttle range
    AP_Float            _batt_current_max;      // current over which maximum throttle is limited
    AP_Float            _batt_current_time_constant;    // Time constant used to limit the maximum current
    AP_Int16            _pwm_min;               // minimum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's min pwm used)
    AP_Int16            _pwm_max;               // maximum PWM value that will ever be output to the motors (if 0, vehicle's throttle input channel's max pwm used)
    AP_Float            _throttle_hover;        // estimated throttle required to hover throttle in the range 0 ~ 1
    AP_Int8             _disarm_disable_pwm;    // disable PWM output while disarmed

    // Maximum lean angle of yaw servo in degrees. This is specific to tricopter
    AP_Float            _servo_angle_max_deg;

    // time to spool motors to min throttle
    AP_Float            _spool_up_time;
    AP_Float            _spool_down_time;

    // motor output variables
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];    // true if motor is enabled

    // battery voltage, current and air pressure compensation variables
    float               _throttle_limit;        // ratio of throttle limit between hover and maximum
    float               _throttle_thrust_max;   // the maximum allowed throttle thrust 0.0 to 1.0 in the range throttle_min to throttle_max
    float               _disarm_safe_timer;     // Timer for the esc when transitioning between zero pwm to minimum
    float               _spin_up_ratio;

    // array of motor output values
    float _actuator[AP_MOTORS_MAX_NUM_MOTORS];


};
