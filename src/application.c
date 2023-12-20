// Tower Kit documentation https://tower.hardwario.com/
// SDK API description https://sdk.hardwario.com/
// Forum https://forum.hardwario.com/

/*
TODO list:
- 1-wire thermometer configuration
- configurable names for all entities
- ADC calibration
- ADC periodic temperature readings
- status LEDs via Adafruit PCA9685 PWM driver
- save config and calibration data to EEPROM
- load config and calibration data from EEPROM
- automatic FAN calibration
- performance profiles for FANs
- MQTT - reporing temperatures, FAN speeds
- MQTT - reporting alarms (unusually high temperature, FAN failure, start/stop calibration, ...)
- MQTT - commands to set profile, start calibration, ...
- MQTT - set config values
*/

#include <application.h>

// Application version
#define FW_VERSION "1.0"
#define EEPROM_SIGNATURE        0x57434330
#define EEPROM_VERSION          0x01

// #define DEBUG   true   // not needed, DEBUG is defined in ninja build file

#ifdef DEBUG
// #define DEBUG_FAN_CALIBRATION   true
// #define DEBUG_ONEWIRE           true
// #define DEBUG_ADC               true
#define DEBUG_ADC_CALIBRATION   true
#endif

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

// ADC configuration
#define ADC_CHANNEL_COUNT 6
// #define ADC_CHANNEL_COUNT 4

// 1-wire configuration
# define OW_MAX_SLAVES 8

// configuration structure
typedef struct {
    uint8_t version;
    uint8_t fans;
    uint16_t fan_ramp_up_step_time; // time in ms between 2 steps of FAN speed ramp up
    uint16_t fan_ramp_down_step_time; // time in ms between 2 steps of FAN speed ramp down
    uint16_t pump_ramp_up_step_time; // time in ms between 2 steps of pump speed ramp up
    uint16_t pump_ramp_down_step_time; // time in ms between 2 steps of pump speed ramp down
} config_t;

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

// ADC HW configuration
typedef struct {
    uint8_t adc_port;
} adc_config_t;

// ADC calibration data
typedef struct {
    bool present;
    bool calibrated;
    float offset;
    float gain;
} adc_calibration_t;

// user defined values for FAN configuration
typedef struct {
    float min_speed;    // user defined minimum speed (0.0 - 1.0), will be reset during calibration
    float max_speed;    // user defined maximum speed (0.0 - 1.0), will be reset during calibration
    bool is_pump;       // flag indicating that this FAN is actually a water pump - will have enforced minimum speed after calibration
} fan_user_config_t;

typedef struct {
    uint32_t signature;
    config_t config;
    fan_config_t fan_config[MAX_FANS];
    fan_user_config_t fan_user_config[MAX_FANS];
    fan_calibration_t fan_calibration[MAX_FANS];
    adc_config_t adc_config[ADC_CHANNEL_COUNT];
    adc_calibration_t adc_calibration[ADC_CHANNEL_COUNT];
} eeprom_t;

eeprom_t eeprom = {
    .signature = EEPROM_SIGNATURE,
    .config = {
        .version = EEPROM_VERSION,
        .fans = 5,
        .fan_ramp_up_step_time = 6000 / FAN_PWM_MAX,        // 6 seconds to ramp up from 0 to 100% speed
        .fan_ramp_down_step_time = 30000 / FAN_PWM_MAX,     // 30 seconds to ramp up from 100% to 0% speed
        .pump_ramp_up_step_time = 4000 / FAN_PWM_MAX,       // 4 seconds to ramp up from 0 to 100% speed
        .pump_ramp_down_step_time = 30000 / FAN_PWM_MAX,    // 30 seconds to ramp up from 100% to 0% speed
    },
    .fan_config = {
        { .gpio_port = TWR_GPIO_P9, .pwm_port = TWR_PWM_P6},
        // { .gpio_port = TWR_GPIO_P10, .pwm_port = TWR_PWM_P7},
        { .gpio_port = TWR_GPIO_P17, .pwm_port = TWR_PWM_P7},
        // { .gpio_port = TWR_GPIO_P11, .pwm_port = TWR_PWM_P8},
        { .gpio_port = TWR_GPIO_P16, .pwm_port = TWR_PWM_P8},
        { .gpio_port = TWR_GPIO_P13, .pwm_port = TWR_PWM_P12},
        { .gpio_port = TWR_GPIO_P15, .pwm_port = TWR_PWM_P14},
    },
    .fan_user_config = {
        { .min_speed = 0.2, .max_speed = 1.0, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .is_pump = true},     // last FAN port is preconfigured as water pump
    },
    .fan_calibration = {
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
    },
    .adc_config = {
        { .adc_port = TWR_ADC_CHANNEL_A0},
        { .adc_port = TWR_ADC_CHANNEL_A1},
        { .adc_port = TWR_ADC_CHANNEL_A2},
        { .adc_port = TWR_ADC_CHANNEL_A3},
        { .adc_port = TWR_ADC_CHANNEL_A4},
        { .adc_port = TWR_ADC_CHANNEL_A5},
    },
    .adc_calibration = {
        { .present = true, .calibrated = false, .offset = 0, .gain = 0},
        { .present = true, .calibrated = false, .offset = 0, .gain = 0},
        { .present = true, .calibrated = false, .offset = 0, .gain = 0},
        { .present = true, .calibrated = false, .offset = 0, .gain = 0},
        { .present = true, .calibrated = false, .offset = 0, .gain = 0},
        { .present = true, .calibrated = false, .offset = 0, .gain = 0},
    },
};

config_t *config = &eeprom.config;
fan_config_t *fan_config = eeprom.fan_config;
fan_calibration_t *fan_calibration = eeprom.fan_calibration;
fan_user_config_t *fan_user_config = eeprom.fan_user_config;
adc_config_t *adc_config = eeprom.adc_config;
adc_calibration_t *adc_calibration = eeprom.adc_calibration;

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
} fan_runtime_t;

fan_runtime_t fan_runtime[MAX_FANS] = {
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm_target = 0, .rpm = 0},
};

// 1-wire thermometer runtime data
typedef struct {
    bool enabled;
    float temperature;
} ow_runtime_t;

uint64_t ow_slave_list[OW_MAX_SLAVES];
uint8_t ow_slave_count = 0;
twr_ds2484_t ds2482;
twr_onewire_t ow;
ow_runtime_t ow_runtime[OW_MAX_SLAVES] = {
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
};

// ADC runtime data
typedef struct {
    bool enabled;
    float temperature;
    uint16_t raw;
} adc_runtime_t;

adc_runtime_t adc_runtime[ADC_CHANNEL_COUNT] = {
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
    { .enabled = false, .temperature = NAN},
};

// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

// Thermometer instance
twr_tmp112_t tmp112;
uint16_t button_click_count = 0;

// helper variables for FAN control
uint8_t fan_list[MAX_FANS] = {0};

// ==== FAN RPM reading ====

/*
    odd ring buffer addresses counts ticks for GPIO = 0,
    even addresses counts ticks for GPIO = 1
*/
#define RPM_RING_LEN 8
uint16_t rpm_ring[MAX_FANS][RPM_RING_LEN];
uint8_t rpm_ring_pos[MAX_FANS] = {0};

// this is called every 1 ms by SDK
void HAL_SYSTICK_Callback(void)
{
    for (uint8_t f = 0; f < config->fans; f++)
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
uint16_t get_rpm(uint8_t fan)
{
    uint16_t count = 0;
    for (uint8_t i = 0; i < RPM_RING_LEN; i++)
    {
        // do not use last 2 positions in buffer we're currently ticking in, it would introduce significant measurement error
        if (i != rpm_ring_pos[fan] && i != (rpm_ring_pos[fan] - 1) % RPM_RING_LEN)
            count += rpm_ring[fan][i];
    }
    if (count == 0)
        return 0;

    // there are 2 ticks per spin, 1 tick is 2 buffer positions, sampled @ 1kHz and we skipped counting last 2 buffer positions
    return (RPM_RING_LEN - 2) * (1000 / 4) * 60 / count;
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

// ==== 1-wire with DS2482 ====
// initiate temperature conversion on all 1-wire devices
void ow_start_conversion()
{
    if (!twr_onewire_reset(&ow))
    {
#ifdef DEBUG_ONEWIRE
        twr_log_debug("ow_start_conversion(): 1-wire not responding");
#endif
        return;
    }
    twr_onewire_skip_rom(&ow);
    twr_onewire_write_byte(&ow, 0x44);
}

// read temperature from 1-wire device
float ow_read_temperature(uint64_t * device_id)
{
    int16_t raw;
    if (!twr_onewire_reset(&ow))
    {
#ifdef DEBUG_ONEWIRE
        twr_log_debug("ow_read_temperature(%016llx): 1-wire not responding", *device_id);
#endif
        return NAN;
    }
    twr_onewire_select(&ow, device_id);
    twr_onewire_write_byte(&ow, 0xBE);
    raw = twr_onewire_read_byte(&ow);
    raw |= (uint16_t)twr_onewire_read_byte(&ow) << 8;
    float temperature = (float)raw / 16.0;
#ifdef DEBUG_ONEWIRE
    twr_log_debug("ow_read_temperature(%016llx): raw: %04x temperature: %.2f °C", *device_id, raw, temperature);
#endif
    return temperature;
}

// read all temperatures from 1-wire devices
void ow_read_temperatures()
{
    for (uint8_t i = 0; i < ow_slave_count; i++)
    {
        if (ow_runtime[i].enabled)
            ow_runtime[i].temperature = ow_read_temperature(&ow_slave_list[i]);
    }
}

// callback to periodically read temperatures from 1-wire devices
void ow_read_temperatures_task(void * param)
{
    ow_read_temperatures();
    ow_start_conversion();
    twr_scheduler_plan_current_relative(1000);
}

void ow_init(void)
{
    static bool initialized = false;
    if (initialized)
        return;
    initialized = true;
    twr_ds2484_init(&ds2482, TWR_I2C_I2C0);
    twr_onewire_ds2484_init(&ow, &ds2482);
    // enumerate all slaves on 1-wire bus
#ifdef DEBUG_ONEWIRE
    twr_log_debug("ow_init(): 1-wire bus scan started");
#endif
    ow_slave_count = twr_onewire_search_all(&ow, ow_slave_list, sizeof(ow_slave_list));
    for (uint8_t i = 0; i < ow_slave_count; i++)
    {
        ow_runtime[i].enabled = true;
        ow_runtime[i].temperature = NAN;
    }
#ifdef DEBUG_ONEWIRE
    twr_log_debug("ow_init(): 1-wire bus scan found %i devices", ow_slave_count);
    for (uint8_t i = 0; i < ow_slave_count; i++)
    {
        twr_log_debug("ow_init(): 1-wire bus scan found device %i: %016llx", i, ow_slave_list[i]);
    }
#endif
    // start periodic task to read temperatures from 1-wire devices
    twr_scheduler_register(ow_read_temperatures_task, NULL, twr_tick_get() + 1000);
}


// ==== ADC thermometers ====
// event handler for ADC thermometer
void adc_event_handler(twr_adc_channel_t channel, twr_adc_event_t event, void *event_param)
{
    if (event == TWR_ADC_EVENT_DONE)
    {
        uint16_t value;
        float temperature = 0;
        if (twr_adc_async_get_value(channel, &value))
        {
            // convert ADC value to temperature
            if (adc_calibration[channel].calibrated)
                temperature = (float)value * adc_calibration[channel].gain + adc_calibration[channel].offset;
            else
                temperature = NAN;
            // temporary for debugging
            // twr_adc_async_get_voltage(channel, &temperature);
            adc_runtime[channel].raw = value;
            adc_runtime[channel].temperature = temperature;
#ifdef DEBUG_ADC
            twr_log_debug("adc_event_handler(): ADC channel %i, value=%d, temperature: %.2f °C", channel, value, temperature);
#endif
        }
        // start next conversion
        for (uint8_t i = channel+1; i < ADC_CHANNEL_COUNT; i++)
        {
            if (adc_runtime[i].enabled){
                twr_adc_async_measure(i);
                break;
            }
        }
    }
}

// initialize ADC
void adc_init(void)
{
    static bool initialized = false;
    if (initialized)
        return;
    initialized = true;
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        if (adc_calibration[i].present)
        {
            adc_runtime[i].enabled = true;
            adc_runtime[i].temperature = NAN;
        }
    }
    twr_adc_init();
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        if (adc_calibration[i].present)
        {
            twr_adc_set_event_handler(i, adc_event_handler, NULL);
            twr_adc_oversampling_set(i, TWR_ADC_OVERSAMPLING_256);
            twr_adc_resolution_set(i, TWR_ADC_RESOLUTION_12_BIT);
        }
    }
}

// ADC thermometers calibration
/*
We will have ring buffer of ADC values for each ADC channel and 1-wire thermometers.
Once we detect stable values in ring buffer, we will compute average value from 1-wire thermometers and use it as calibration value.
Then we will repeat this process, but wait for at least ADC_CALIBRATION_TEMP_DELTA degrees temperature change.
Calibration points will be stored in adc_calibration_measure_point[] array.
When we have all calibration points, we will compute ADC calibration data (offset and gain) and store it in adc_calibration[] array.
*/
#define ADC_CALIBRATION_RING_LEN 8
#define ADC_CALIBRATION_TEMP_DELTA 6.0
#define ADC_CALIBRATION_TEMP_STABLE 1.0
// #define ADC_CALIBRATION_STEP_INTERVAL 5000
#define ADC_CALIBRATION_STEP_INTERVAL 1000
#define ADC_CALIBRATION_POINTS 2
// TODO adjust sane values from real measurements
#define ADC_SANE_LOW 6000
#define ADC_SANE_HIGH 58000

typedef struct 
{
    float avg_temperature;  // average temperature from 1-wire thermometers
    uint16_t adc_value[ADC_CHANNEL_COUNT]; // ADC values for each ADC channel
} adc_calibration_measure_t;

adc_calibration_measure_t adc_calibration_measure_ring[ADC_CALIBRATION_RING_LEN];
uint8_t adc_calibration_measure_ring_pos = 0;
adc_calibration_measure_t adc_calibration_measure_point[ADC_CALIBRATION_POINTS];
uint8_t adc_calibration_measure_point_pos = 0;
float adc_last_calibration_temperature = -999.0; 

void adc_compute_calibration(void)
{
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
#ifdef DEBUG_ADC_CALIBRATION
        twr_log_debug("adc_compute_calibration(): ADC channel %i, Point 0: Temperature=%.2f, ADC Value=%u, Point 1: Temperature=%.2f, ADC Value=%u", i, 
                      adc_calibration_measure_point[0].avg_temperature, adc_calibration_measure_point[0].adc_value[i],
                      adc_calibration_measure_point[1].avg_temperature, adc_calibration_measure_point[1].adc_value[i]);
#endif
        // check for sane values and set .present flag
        if (adc_calibration_measure_point[0].adc_value[i] < ADC_SANE_LOW || adc_calibration_measure_point[0].adc_value[i] > ADC_SANE_HIGH ||
            adc_calibration_measure_point[1].adc_value[i] < ADC_SANE_LOW || adc_calibration_measure_point[1].adc_value[i] > ADC_SANE_HIGH)
        {
            adc_calibration[i].present = false;
            adc_calibration[i].calibrated = false;
            adc_runtime[i].enabled = false;
            continue;
        }

        // compute gain
        adc_calibration[i].gain = (adc_calibration_measure_point[1].avg_temperature - adc_calibration_measure_point[0].avg_temperature)
                                    / (float)(adc_calibration_measure_point[1].adc_value[i] - adc_calibration_measure_point[0].adc_value[i]);
        // compute offset
        adc_calibration[i].offset = adc_calibration_measure_point[0].avg_temperature - (float)adc_calibration_measure_point[0].adc_value[i] * adc_calibration[i].gain;
        adc_calibration[i].present = true;
        adc_calibration[i].calibrated = true;
        adc_runtime[i].enabled = true;
#ifdef DEBUG_ADC_CALIBRATION
        twr_log_debug("adc_compute_calibration(): ADC channel %i, gain=%.5f, offset=%.2f", i, adc_calibration[i].gain, adc_calibration[i].offset);
#endif
    }
}

// callback to periodically read temperatures from 1-wire devices and ADC channels
void adc_calibration_callback(void * param) {
    static bool buffer_full = false;
    float avg_temperature = 0.0;
    uint8_t avg_temperature_count = 0;
    for (uint8_t i = 0; i < ow_slave_count; i++)
    {
        if (ow_runtime[i].enabled && !isnan(ow_runtime[i].temperature))
        {
            avg_temperature += ow_runtime[i].temperature;
            avg_temperature_count++;
        }
    }
    if (avg_temperature_count > 0)
        avg_temperature /= (float)avg_temperature_count;
    else
        avg_temperature = NAN;
    adc_calibration_measure_ring[adc_calibration_measure_ring_pos].avg_temperature = avg_temperature;
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        adc_calibration_measure_ring[adc_calibration_measure_ring_pos].adc_value[i] = adc_runtime[i].raw;
    }
    if (++adc_calibration_measure_ring_pos >= ADC_CALIBRATION_RING_LEN)
        buffer_full = true;
    adc_calibration_measure_ring_pos %= ADC_CALIBRATION_RING_LEN;
    // check if we have stable values in ring buffer
    bool stable = buffer_full;
    float tmin = 999.0;
    float tmax = -999.0;
    for (uint8_t i = 0; i < ADC_CALIBRATION_RING_LEN; i++)
    {
        if (isnan(adc_calibration_measure_ring[i].avg_temperature))
        {
            stable = false;
            break;
        }
        if (adc_calibration_measure_ring[i].avg_temperature < tmin)
            tmin = adc_calibration_measure_ring[i].avg_temperature;
        if (adc_calibration_measure_ring[i].avg_temperature > tmax)
            tmax = adc_calibration_measure_ring[i].avg_temperature;
    }
    stable &= (tmax - tmin) < ADC_CALIBRATION_TEMP_STABLE;
#ifdef DEBUG_ADC_CALIBRATION
    twr_log_debug("adc_calibration_callback(): stable=%i, tmin=%.2f, tmax=%.2f", stable, tmin, tmax);
#endif
    if (stable)
    {
        // check if temperature changed enough since last calibration point
        if (isnan(adc_last_calibration_temperature) || fabs(adc_last_calibration_temperature - adc_calibration_measure_ring[adc_calibration_measure_ring_pos].avg_temperature) > ADC_CALIBRATION_TEMP_DELTA)
        {
            // compute average values from ring buffer
            adc_calibration_measure_t avg;
            memset(&avg, 0, sizeof(avg));
            for (uint8_t i = 0; i < ADC_CALIBRATION_RING_LEN; i++)
            {
                avg.avg_temperature += adc_calibration_measure_ring[i].avg_temperature;
                for (uint8_t j = 0; j < ADC_CHANNEL_COUNT; j++)
                {
// #ifdef DEBUG_ADC_CALIBRATION
//                     twr_log_debug("adc_calibration_callback(): ADC calibration point %i, ADC %i: Temperature=%.2f, ADC Value=%u", i, j, adc_calibration_measure_ring[i].avg_temperature, adc_calibration_measure_ring[i].adc_value[j]);
// #endif                    
                    avg.adc_value[j] += adc_calibration_measure_ring[i].adc_value[j] / ADC_CALIBRATION_RING_LEN;
                }
            }
            avg.avg_temperature /= (float)ADC_CALIBRATION_RING_LEN;
#ifdef DEBUG_ADC_CALIBRATION
            for (uint8_t j = 0; j < ADC_CHANNEL_COUNT; j++)
            {
                twr_log_debug("adc_calibration_callback(): ADC calibration point %i: Temperature=%.2f, ADC Value=%u", adc_calibration_measure_point_pos, avg.avg_temperature, avg.adc_value[j]);
            }
#endif                
            // store calibration point
            adc_calibration_measure_point[adc_calibration_measure_point_pos] = avg;
            adc_last_calibration_temperature = avg.avg_temperature;
            if (++adc_calibration_measure_point_pos >= ADC_CALIBRATION_POINTS)
            {
                adc_compute_calibration();
            } else {
                // start next calibration step
                twr_scheduler_plan_current_relative(ADC_CALIBRATION_STEP_INTERVAL);
            }
            return;
        }
        // compute average values from ring buffer
        float avg_adc_value[ADC_CHANNEL_COUNT] = {0.0};
        for (uint8_t i = 0; i < ADC_CALIBRATION_RING_LEN; i++)
        {
            for (uint8_t j = 0; j < ADC_CHANNEL_COUNT; j++)
            {
                avg_adc_value[j] += (float)adc_calibration_measure_ring[i].adc_value[j];
            }
        }
    }
    twr_scheduler_plan_current_relative(ADC_CALIBRATION_STEP_INTERVAL);
}

// start ADC calibration
void adc_calibration_start(void)
{
    adc_calibration_measure_point_pos = 0;
    adc_calibration_measure_ring_pos = 0;
    adc_last_calibration_temperature = -999.0;
    memset(adc_calibration_measure_point, 0, sizeof(adc_calibration_measure_point));
    memset(adc_calibration_measure_ring, 0, sizeof(adc_calibration_measure_ring));
    twr_scheduler_register(adc_calibration_callback, NULL, twr_tick_get() + ADC_CALIBRATION_STEP_INTERVAL);
}


#define FAN_SPEED_STEP 0.6
// Button event callback
void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    static float fan_speed = 0.6;
    static float fan_delta = -FAN_SPEED_STEP;
    // Log button event
    twr_log_info("APP: Button event: %i", event);

    // Check event source
    if (event == TWR_BUTTON_EVENT_CLICK)
    {
        // Toggle LED pin state
        twr_led_set_mode(&led, TWR_LED_MODE_TOGGLE);

         // Publish message on radio
        button_click_count++;
        twr_radio_pub_push_button(&button_click_count);

        fan_speed += fan_delta;
        if (fan_speed > 1.0)
        {
            fan_speed = 1.0;
            fan_delta = -FAN_SPEED_STEP;
        }
        if (fan_speed < 0.0)
        {
            fan_speed = 0.0;
            fan_delta = FAN_SPEED_STEP;
        }
        fan_set_speed(0, fan_speed);
    }
}

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *event_param)
{
    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        float celsius;
        // Read temperature
        twr_tmp112_get_temperature_celsius(self, &celsius);

#ifdef DEBUG            
        twr_log_debug("APP: temperature: %.2f °C", celsius);
#endif

        twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &celsius);
    }
}

// Application initialization function which is called once after boot
void application_init(void)
{
    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, 0);
    twr_led_pulse(&led, 2000);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, 0);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize thermometer on core module
    twr_tmp112_init(&tmp112, TWR_I2C_I2C0, 0x49);
    twr_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    twr_tmp112_set_update_interval(&tmp112, 10000);

    // setup PWM for all configured FAN GPIO ports
    for (uint8_t f = 0; f < config->fans; f++)
    {
        fan_list[f] = f;
        _pwm_init(fan_config[f].pwm_port, FAN_PWM_MAX);
        fan_set_speed(f, FAN_SPEED_INIT);
        twr_pwm_enable(fan_config[f].pwm_port);
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("APP: PWM init, FAN=%i, port=%i", f, fan_config[f].pwm_port);
#endif
        twr_gpio_init(fan_config[f].gpio_port);
        twr_gpio_set_mode(fan_config[f].gpio_port, TWR_GPIO_MODE_INPUT);
        twr_gpio_set_pull(fan_config[f].gpio_port, TWR_GPIO_PULL_UP);
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("APP: RPM reading init, FAN=%i, port=%i", f, fan_config[f].gpio_port);
#endif
        // temporary to test calibration
        fan_calibration_start(f);
    }
    twr_system_pll_enable();

    // Initialize 1-wire bus
    ow_init();
    ow_start_conversion();

    // Initialize ADC
    adc_init();
    adc_calibration_start();

    // Initialize radio
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);
    // Send radio pairing request
    twr_radio_pairing_request("water-cooler", FW_VERSION);
}

// Application task function (optional) which is called peridically if scheduled
void application_task(void)
{
    static int counter = 0;
    counter++;

    // Log task run and increment counter
    // twr_log_debug("APP: Task run (count: %d)", ++counter);
    twr_log_debug("APP: RPM: %d", get_rpm(0));
    float tmin = NAN;
    float tmax = NAN;
    float tavg = 0.0;
    for (uint8_t t = 0; t < ow_slave_count; t++)
    {
        if (ow_runtime[t].enabled) {
            if (isnan(tmin) || ow_runtime[t].temperature < tmin)
                tmin = ow_runtime[t].temperature;
            if (isnan(tmax) || ow_runtime[t].temperature > tmax)
                tmax = ow_runtime[t].temperature;
            tavg += ow_runtime[t].temperature;
        }
    }
    tavg /= (float)ow_slave_count;
    twr_log_debug("APP: 1-wire thermometers: %i devices, min=%.2f, max=%.2f, avg=%.2f, delta=%.2f", ow_slave_count, tmin, tmax, tavg, tmax-tmin);
    uint8_t adc_count = 0;
    tmin = NAN;
    tmax = NAN;
    tavg = 0.0;
    for (uint8_t t = 0; t < ADC_CHANNEL_COUNT; t++)
    {
        if (adc_runtime[t].enabled) {
            twr_log_debug("APP: ADC thermometer %i: %.2f (raw=%i)", t, adc_runtime[t].temperature, adc_runtime[t].raw);
            if (isnan(tmin) || adc_runtime[t].temperature < tmin)
                tmin = adc_runtime[t].temperature;
            if (isnan(tmax) || adc_runtime[t].temperature > tmax)
                tmax = adc_runtime[t].temperature;
            tavg += adc_runtime[t].temperature;
            adc_count++;
        }
    }
    if (adc_count > 0) {
        tavg /= (float)adc_count;
        twr_log_debug("APP: ADC thermometers: %i devices, min=%.2f, max=%.2f, avg=%.2f, delta=%.2f", adc_count, tmin, tmax, tavg, tmax-tmin);
    }
    // for (uint8_t t = 0; t < ADC_CHANNEL_COUNT; t++)
    // {
    //     if (adc_runtime[t].enabled) {
    //         twr_log_debug("APP: ADC thermometer %i: %.2f", t, adc_runtime[t].temperature);
    //     }
    // }
    twr_adc_async_measure(TWR_ADC_CHANNEL_A0);

    // Plan next run of this task in 1000 ms
    twr_scheduler_plan_current_from_now(10000);
}
