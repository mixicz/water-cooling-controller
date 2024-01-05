#include <application.h>
#include <fan.h>

const fan_config_t fan_config[MAX_FANS] = {
    { .gpio_port = TWR_GPIO_P9, .pwm_port = TWR_PWM_P6},
    { .gpio_port = TWR_GPIO_P10, .pwm_port = TWR_PWM_P7},
    { .gpio_port = TWR_GPIO_P11, .pwm_port = TWR_PWM_P8},
    { .gpio_port = TWR_GPIO_P13, .pwm_port = TWR_PWM_P12},
    { .gpio_port = TWR_GPIO_P15, .pwm_port = TWR_PWM_P14},
};

fan_runtime_t fan_runtime[MAX_FANS] = {
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
};

// helper variables for FAN control
uint8_t fan_list[MAX_FANS] = {0, 1, 2, 3, 4};


// ==== FAN RPM reading ====

/*
    odd ring buffer addresses counts ticks for GPIO = 0,
    even addresses counts ticks for GPIO = 1
*/
#define RPM_RING_LEN 8
uint16_t rpm_ring[MAX_FANS][RPM_RING_LEN];
uint8_t rpm_ring_pos[MAX_FANS] = {0};

// this MUST be called from HAL_SYSTICK_Callback() in SDK
void fan_HAL_SYSTICK_Callback(void)
{
    for (uint8_t f = 0; f < MAX_FANS; f++)
    {
        // if input changed state, switch to next ring position
        if (twr_gpio_get_input(fan_config[f].gpio_port) != rpm_ring_pos[f] % 2)
        {
            rpm_ring_pos[f]++;
            rpm_ring_pos[f] %= RPM_RING_LEN;
            rpm_ring[f][rpm_ring_pos[f]] = 0;
        }
        rpm_ring[f][rpm_ring_pos[f]]++;

        // check for non-rotating FAN - if we detect no change in RPM signal for at least RPM_TIMEOUT ms, then we zero up the whole ring
        if (rpm_ring[f][rpm_ring_pos[f]] > RPM_TIMEOUT) {
            for (int i=0; i<RPM_RING_LEN; i++)
                rpm_ring[f][i] = 0;
        }
    }
}

// compute RPM from tick count in ring buffer
// this might be called periodically in order to keep fan_runtime[fan].rpm up to date
uint16_t get_rpm(uint8_t fan)
{
    uint16_t count = 0;
    for (uint8_t i = 0; i < RPM_RING_LEN; i++)
    {
        // do not use last 2 positions in buffer we're currently ticking in, it would introduce significant measurement error
        if (i != rpm_ring_pos[fan] && i != (rpm_ring_pos[fan] - 1) % RPM_RING_LEN)
            count += rpm_ring[fan][i];
    }
    if (count == 0) {
        // no ticks detected, FAN is not rotating
        fan_runtime[fan].rpm = 0;
        fan_runtime[fan].last_rpm_tick = twr_tick_get();
    } else {
        // compute RPM from tick count
        fan_runtime[fan].rpm = (RPM_RING_LEN - 2) * (1000 / 4) * 60 / count;
        fan_runtime[fan].last_rpm_tick = twr_tick_get();
    }

    // there are 2 ticks per spin, 1 tick is 2 buffer positions, sampled @ 1kHz and we skipped counting last 2 buffer positions
    return fan_runtime[fan].rpm;
}

// ==== FAN PWM control ====

// Custom function for PWM initialization to enable higher PWM frequency (for PC fans is 25 kHz recommended)
void _pwm_init(twr_pwm_channel_t channel, int32_t pwm_max)
{
    static bool tim2_initialized = false;
    static bool tim3_initialized = false;
    static bool tim21_initialized = false;
    static bool pll_enabled = false;

    if (!pll_enabled)
    {
        twr_system_pll_enable();
        pll_enabled = true;
    }

    if (!tim2_initialized && (channel == TWR_PWM_P0 || channel == TWR_PWM_P1 || channel == TWR_PWM_P2 || channel == TWR_PWM_P3))
    {
        twr_pwm_tim_configure(TWR_PWM_TIM2_P0_P1_P2_P3, 1, pwm_max);
        tim2_initialized = true;
    }

    if (!tim3_initialized && (channel == TWR_PWM_P6 || channel == TWR_PWM_P7 || channel == TWR_PWM_P8))
    {
        twr_pwm_tim_configure(TWR_PWM_TIM3_P6_P7_P8, 1, pwm_max);
        tim3_initialized = true;
    }

    if (!tim21_initialized && (channel == TWR_PWM_P12 || channel == TWR_PWM_P14))
    {
        twr_pwm_tim_configure(TWR_PWM_TIM21_P12_P14, 1, pwm_max);
        tim21_initialized = true;
    }
}

// return ramp up/down step time in ms
uint16_t fan_step_time(uint8_t fan, bool up)
{
    if (fan_user_config[fan].is_pump) {
        if (up)
            return config->pump_ramp_up_step_time;
        else
            return config->pump_ramp_down_step_time;
    } else {
        if (up)
            return config->fan_ramp_up_step_time;
        else
            return config->fan_ramp_down_step_time;
    }
}

// FAN ramp up/down step
void fan_ramp_step(void * ptr_fan)
{
    uint8_t fan = *(uint8_t*)ptr_fan;
    int direction = 0;
    if (fan_runtime[fan].pwm_current == fan_runtime[fan].pwm_target) {
        fan_runtime[fan].ramp_in_progress = false;
        twr_scheduler_unregister(twr_scheduler_get_current_task_id());
        return;
    } else if (fan_runtime[fan].pwm_current < fan_runtime[fan].pwm_target) {
        fan_runtime[fan].pwm_current++;
        direction = 1;
    } else {
        fan_runtime[fan].pwm_current--;
        direction = -1;
    }
#ifdef DEBUG_FAN_CALIBRATION
    twr_log_debug("fan_ramp_step(): FAN=%i, PWM=%i, direction=%i", fan, fan_runtime[fan].pwm_current, direction);
#endif 
    twr_pwm_set(fan_config[fan].pwm_port, fan_runtime[fan].pwm_current);
    twr_scheduler_plan_current_relative(fan_step_time(fan, direction > 0));
}

// FAN set speed using calibration data
void fan_set_speed(uint8_t fan, float speed)
{
#ifdef DEBUG_FAN_CALIBRATION            
    twr_log_debug("fan_set_speed(): FAN=%i, requested: speed=%.2f", fan, speed);
#endif
    if (fan_runtime[fan].enabled == false || fan_runtime[fan].pwm_control == false || fan_runtime[fan].calibration_in_progress == true)
        return;
    if (speed < fan_user_config[fan].min_speed)
        speed = fan_user_config[fan].min_speed;
    if (speed > fan_user_config[fan].max_speed)
        speed = fan_user_config[fan].max_speed;
    if (fan_calibration[fan].calibrated == true) {
        float index = (float)(FAN_CALIBRATION_STEPS-1) * speed;
        float weight = index - (int)index;
        if ((int)index >= FAN_CALIBRATION_STEPS-1)
            fan_runtime[fan].pwm_target = fan_calibration[fan].points[FAN_CALIBRATION_STEPS-1];
        else if ((int)index < 0)
            fan_runtime[fan].pwm_target = fan_calibration[fan].points[0];
        else
            // linear interpolation between 2 points in calibration data (PWM values)
            fan_runtime[fan].pwm_target = (uint16_t)((float)fan_calibration[fan].points[(int)index] * (1.0 - weight) + (float)fan_calibration[fan].points[(int)index+1] * weight);
        // cap PWM value to minimum PWM value as detected by calibration process
        if (fan_runtime[fan].pwm_target < fan_calibration[fan].min_pwm)
            fan_runtime[fan].pwm_target = fan_calibration[fan].min_pwm;
    } else {
        fan_runtime[fan].pwm_target = (uint16_t)((float)FAN_PWM_MAX * speed);
        // cap PWM value to minimum PWM value according to specification
        if (fan_runtime[fan].pwm_target < FAN_PWM_MIN)
            fan_runtime[fan].pwm_target = FAN_PWM_MIN;
    }
    // cap PWM value to maximum PWM value
    if (fan_runtime[fan].pwm_target > FAN_PWM_MAX)
        fan_runtime[fan].pwm_target = FAN_PWM_MAX;
#ifdef DEBUG_FAN_CALIBRATION
    twr_log_debug("fan_set_speed(): FAN=%i, computed: speed=%.2f, PWM=%i -> %i", fan, speed, fan_runtime[fan].pwm_current, fan_runtime[fan].pwm_target);
#endif
    if (fan_runtime[fan].pwm_current == fan_runtime[fan].pwm_target)
        return;
    // TODO: stat ramp up/down callback
    // twr_pwm_set(fan_config[fan].pwm_port, fan_runtime[fan].pwm_target);
    if (fan_runtime[fan].ramp_in_progress == false) {
        fan_runtime[fan].ramp_in_progress = true;
        twr_scheduler_register(fan_ramp_step, &fan_list[fan], twr_tick_get() + fan_step_time(fan, fan_runtime[fan].pwm_target > fan_runtime[fan].pwm_current));
    }
}

// ==== FAN calibration ====

// FAN calibration
struct fan_calibration_measure_t
{
    uint16_t pwm;
    uint16_t rpm;
};

struct fan_calibration_measure_t fan_calibration_measure[MAX_FANS][FAN_CALIBRATION_STEPS];

// compute calibration data from measured values
void fan_compute_calibration(uint8_t fan)
{
    uint16_t min_rpm = 0;
    uint16_t max_rpm = 0;

    // preset default sane values according to specification
    fan_user_config[fan].max_speed = 1.0;
    fan_user_config[fan].min_speed = 0.2;
    fan_calibration[fan].min_pwm = FAN_PWM_MIN;

#ifdef DEBUG_FAN_CALIBRATION
    twr_log_debug("fan_compute_calibration(): FAN %i computing calibration data", fan);
#endif

    for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++)
    {
        if (i == 0 || fan_calibration_measure[fan][i].rpm < min_rpm)
            min_rpm = fan_calibration_measure[fan][i].rpm;
        if (i == 0 || fan_calibration_measure[fan][i].rpm > max_rpm)
            max_rpm = fan_calibration_measure[fan][i].rpm;
    }

    // if FAN is not connected, max_rpm will be 0
    if (max_rpm == 0)
    {
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("fan_compute_calibration(): FAN %i not connected, min_rpm=%i, max_rpm=%i", fan, min_rpm, max_rpm);
#endif
        fan_calibration[fan].calibrated = false;
        fan_calibration[fan].pwm_capable = false;   
        fan_calibration[fan].present = false;
        return;
    }
    fan_calibration[fan].present = true;

    // if FAN is not capable of PWM control, max_rpm will be close to min_rpm
    uint16_t rpm_range = max_rpm - min_rpm;
    if ((float)rpm_range / (float)max_rpm < 0.2)
    {
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("fan_compute_calibration(): FAN %i not capable of PWM control, min_rpm=%i, max_rpm=%i", fan, min_rpm, max_rpm);
#endif
        fan_calibration[fan].calibrated = true;
        fan_calibration[fan].pwm_capable = false;   
        return;
    }
    fan_calibration[fan].pwm_capable = true;   

    // Detect if lower 20% and 10% of PWM range is usable
    int pwm20_min_rpm = 0x7FFFFFFF;
    int pwm20_max_rpm = -1;
    int pwm10_min_rpm = 0x7FFFFFFF;
    int pwm10_max_rpm = -1;
    for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++)
    {
        if (fan_calibration_measure[fan][i].pwm <= FAN_PWM_MAX / 5)
        {
            if (i == FAN_CALIBRATION_STEPS-1 || fan_calibration_measure[fan][i].rpm < pwm20_min_rpm)
                pwm20_min_rpm = fan_calibration_measure[fan][i].rpm;
            if (fan_calibration_measure[fan][i].rpm > pwm20_max_rpm)
                pwm20_max_rpm = fan_calibration_measure[fan][i].rpm;
        }
        if (fan_calibration_measure[fan][i].pwm <= FAN_PWM_MAX / 10)
        {
            if (i == FAN_CALIBRATION_STEPS-1 || fan_calibration_measure[fan][i].rpm < pwm10_min_rpm)
                pwm10_min_rpm = fan_calibration_measure[fan][i].rpm;
            if (fan_calibration_measure[fan][i].rpm > pwm10_max_rpm)
                pwm10_max_rpm = fan_calibration_measure[fan][i].rpm;
        }
    }
#ifdef DEBUG_FAN_CALIBRATION
    twr_log_debug("fan_compute_calibration(): FAN %i: RPM peaks of lower 20%% and 10%% PWM range, min20=%i, max20=%i, min10=%i, max10=%i", fan, pwm20_min_rpm, pwm20_max_rpm, pwm10_min_rpm, pwm10_max_rpm);
#endif
    if ((float)(pwm20_max_rpm - pwm20_min_rpm) / (float)pwm20_max_rpm > 0.6)
    {
        if ((float)(pwm10_max_rpm - pwm10_min_rpm) / (float)pwm10_max_rpm > 0.6) {
#ifdef DEBUG_FAN_CALIBRATION
            twr_log_debug("fan_compute_calibration(): FAN %i: lower 0-20%% of PWM range seems usable", fan);
#endif
            fan_calibration[fan].calibrated = true;
            fan_user_config[fan].min_speed = 0.0;
            fan_calibration[fan].min_pwm = 0;
        } else {
#ifdef DEBUG_FAN_CALIBRATION
            twr_log_debug("fan_compute_calibration(): FAN %i: lower 10-20%% of PWM range seems usable", fan);
#endif
            fan_calibration[fan].calibrated = true;
            fan_user_config[fan].min_speed = 0.1;
            fan_calibration[fan].min_pwm = FAN_PWM_MAX / 10;
        }
        // for pumps set minimum speed to 20% to prevent overheating
        if (fan_user_config[fan].is_pump) {
            fan_user_config[fan].min_speed = 0.2;
            fan_calibration[fan].min_pwm = FAN_PWM_MIN;
        }
    }

#ifdef DEBUG_FAN_CALIBRATION
    float rpm_deviation[FAN_CALIBRATION_STEPS];
#endif
    // Compute calibration points from measured values
    // Note: we're computing calibration points in reverse order as we measured starting with 100% speed, but we want to have 0% speed as first calibration point
    for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++)
    {
        float speed = 1.0 - (float)i / (float)(FAN_CALIBRATION_STEPS-1);
        uint16_t rpm = (uint16_t)((float)max_rpm * speed);
#ifdef DEBUG_FAN_CALIBRATION
        if (fan_calibration_measure[fan][i].rpm > 0) {
            rpm_deviation[i] = (float)(fan_calibration_measure[fan][i].rpm - rpm) / (float)rpm;
        } else {
            rpm_deviation[i] = 0.0;
        }
#endif
        int idx_low = -1;
        int idx_high = -1;
        for (uint8_t j = 0; j < FAN_CALIBRATION_STEPS; j++) {
            if (fan_calibration_measure[fan][j].rpm < rpm)
                idx_low = j;
            if (fan_calibration_measure[fan][j].rpm > rpm) {
                idx_high = j;
                break;
            }
        }
        if (idx_low > -1 && idx_high > -1) {
            // interpolate between 2 points
            uint16_t pwm_low = fan_calibration_measure[fan][idx_low].pwm;
            uint16_t pwm_high = fan_calibration_measure[fan][idx_high].pwm;
            uint16_t rpm_low = fan_calibration_measure[fan][idx_low].rpm;
            uint16_t rpm_high = fan_calibration_measure[fan][idx_high].rpm;
            uint16_t pwm = (uint16_t)((float)pwm_low + (float)(pwm_high - pwm_low) * (float)(rpm - rpm_low) / (float)(rpm_high - rpm_low));
            if (pwm > FAN_PWM_MAX) {
                pwm = FAN_PWM_MAX;
#ifdef DEBUG_FAN_CALIBRATION
                twr_log_debug("fan_compute_calibration(): FAN %i: PWM value overflow, speed=%.2f, pwm=%i, rpm=%i, pwm_low=%i, pwm_high=%i, rpm_low=%i, rpm_high=%i", 
                    fan, speed, pwm, rpm, pwm_low, pwm_high, rpm_low, rpm_high);
#endif
            }
            fan_calibration[fan].points[FAN_CALIBRATION_STEPS-1-i] = pwm;
        } else {
            // Unable to compute this calibration point, fallback to assume linear PWM/RPM dependency
            fan_calibration[fan].points[FAN_CALIBRATION_STEPS-1-i] = (uint16_t)((float)FAN_PWM_MAX * speed);
        }
    }
#ifdef DEBUG_FAN_CALIBRATION
    // debug print of calibration data
    for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++) {
        uint16_t expected_pwm = (uint16_t)((float)FAN_PWM_MAX * (float)i / (float)(FAN_CALIBRATION_STEPS-1));
        float delta;
        if (expected_pwm > 0)
            delta = (float)(fan_calibration[fan].points[i]-expected_pwm) / (float)expected_pwm;
        else
            delta = 0.0;
        twr_log_debug("fan_compute_calibration(): FAN %i computed data: speed=%.2f, pwm=%i, rpm=%i, correction=%1.1f%%, RPM deviation=%1.1f%%", 
            fan, 1.0 - (float)i / (float)(FAN_CALIBRATION_STEPS-1), fan_calibration[fan].points[i], (uint16_t)((float)max_rpm * (1.0 - (float)i / (float)(FAN_CALIBRATION_STEPS-1))), delta * 100.0, rpm_deviation[i] * 100.0);
    }
#endif
}

// FAN calibration step
void fan_calibration_step(void * ptr_fan)
{
    static uint8_t step[MAX_FANS] = {0, 0, 0, 0, 0};
    static uint16_t last_rpm[MAX_FANS] = {0, 0, 0, 0, 0};
    uint8_t fan = *(uint8_t*)ptr_fan;
    float speed = 1.0 - (float)step[fan] / (float)(FAN_CALIBRATION_STEPS-1);
    if (speed < 0.0)
        speed = 0.0;    // to have sane values for 1 last extra step where we compute calibration data
    uint16_t pwm = (uint16_t)((float)FAN_PWM_MAX * speed);
    if (step[fan] < FAN_CALIBRATION_STEPS)
        fan_calibration_measure[fan][step[fan]].pwm = pwm;
    if (step[fan] > 0) {
        uint16_t rpm = get_rpm(fan);
        // TODO: add counter to prevent infinite loop
        if (rpm != last_rpm[fan]) {
#ifdef DEBUG_FAN_CALIBRATION
            twr_log_debug("fan_calibration_step(): FAN %i RPM stabilization step: %i != %i", fan, rpm, last_rpm[fan]);
#endif
            last_rpm[fan] = rpm;
            twr_scheduler_plan_current_relative(FAN_CALIBRATION_TIME_RPM);
            return;
        }
        fan_calibration_measure[fan][step[fan]-1].rpm = rpm;
        last_rpm[fan] = 0;
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("fan_calibration_step(): FAN %i step %d, speed=%.2f (pwm=%d), prev rpm=%i", fan, step[fan], speed, pwm, fan_calibration_measure[fan][step[fan]-1].rpm);
#endif
    } else {
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("fan_calibration_step(): FAN %i step 0, speed=%.2f (pwm=%d)", fan, speed, pwm);
#endif
    }
    step[fan]++;
    if (step[fan] > FAN_CALIBRATION_STEPS) {
        // set FAN speed to previous value
        twr_pwm_set(fan_config[fan].pwm_port, fan_runtime[fan].pwm_current);
        fan_runtime[fan].calibration_in_progress = false;
        step[fan] = 0;
        twr_scheduler_unregister(twr_scheduler_get_current_task_id());
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("fan_calibration_step(): FAN %i calibration done", fan);
        for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++)
            twr_log_debug("fan_calibration_step(): FAN %i measured values: pwm=%i, rpm=%i", fan, fan_calibration_measure[fan][i].pwm, fan_calibration_measure[fan][i].rpm);
#endif
        // compute calibration data from measured values
        fan_compute_calibration(fan);
        // TODO: save calibration data to EEPROM
    } else {
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("fan_calibration_step(): FAN %i set PWM=%i", fan, pwm);
#endif
        twr_pwm_set(fan_config[fan].pwm_port, pwm);
        // increase calibration time for low RPM range due to slower RPM reading
        if (pwm < FAN_PWM_MAX / 4) {
            twr_scheduler_plan_current_relative(FAN_CALIBRATION_TIME_LOW_RPM);
        } else {
            twr_scheduler_plan_current_relative(FAN_CALIBRATION_TIME);
        }
    }
}

// FAN calibration start
void fan_calibration_start(uint8_t fan)
{
#ifdef DEBUG_FAN_CALIBRATION
    twr_log_debug("fan_calibration_start(): FAN %i calibration start", fan);
#endif
    fan_calibration[fan].calibrated = false;
    fan_runtime[fan].calibration_in_progress = true;
    twr_pwm_set(fan_config[fan].pwm_port, FAN_PWM_MAX);
    twr_scheduler_register(fan_calibration_step, &fan_list[fan], twr_tick_get() + FAN_CALIBRATION_TIME_INIT);
}
