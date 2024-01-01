#ifndef _FAN_H
#define _FAN_H

#include <twr.h>

// FAN configuration
#define MAX_FANS 5
#define RPM_TIMEOUT 5000
#define FAN_PWM_FREQ 25000
#define FAN_PWM_MAX (1000000 / FAN_PWM_FREQ)
#define FAN_PWM_MIN (FAN_PWM_MAX / 5)
#define FAN_SPEED_INIT 0.4
#define FAN_CALIBRATION_STEPS 11
#define FAN_CALIBRATION_TIME 3000
#define FAN_CALIBRATION_TIME_LOW_RPM 5000
#define FAN_CALIBRATION_TIME_INIT 8000
#define FAN_CALIBRATION_TIME_RPM 500


// FAN HW configuration
typedef struct {
    uint8_t gpio_port;
    uint8_t pwm_port;
} fan_config_t;

// FAN calibration data
typedef struct {
    bool present;
    bool calibrated;
    bool pwm_capable;
    uint16_t min_pwm;   // minimum PWM value as detected by calibration process
    uint16_t points[FAN_CALIBRATION_STEPS]; // PWM values for calibration steps, index 0 is for 0% speed, index FAN_CALIBRATION_STEPS-1 is for 100% speed
} fan_calibration_t;

// user defined values for FAN configuration
typedef struct {
    float min_speed;    // user defined minimum speed (0.0 - 1.0), will be reset during calibration
    float max_speed;    // user defined maximum speed (0.0 - 1.0), will be reset during calibration
    bool is_pump;       // flag indicating that this FAN is actually a water pump - will have enforced minimum speed after calibration
} fan_user_config_t;

// FAN runtime data
typedef struct {
    bool enabled;
    bool calibrated;
    bool pwm_control;
    bool calibration_in_progress;
    bool ramp_in_progress;
    uint16_t pwm_target;
    uint16_t pwm_current;
    uint16_t rpm;
    twr_tick_t last_rpm_tick;
} fan_runtime_t;

extern fan_runtime_t fan_runtime[MAX_FANS];

void fan_HAL_SYSTICK_Callback(void);
uint16_t get_rpm(uint8_t fan);
void _pwm_init(twr_pwm_channel_t channel, int32_t pwm_max);
uint16_t fan_step_time(uint8_t fan, bool up);
void fan_ramp_step(void * ptr_fan);
void fan_set_speed(uint8_t fan, float speed);
void fan_compute_calibration(uint8_t fan);
void fan_calibration_step(void * ptr_fan);
void fan_calibration_start(uint8_t fan);

#endif