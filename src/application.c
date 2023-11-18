// Tower Kit documentation https://tower.hardwario.com/
// SDK API description https://sdk.hardwario.com/
// Forum https://forum.hardwario.com/

#include <application.h>

// Application version
#define FW_VERSION "1.0"
#define EEPROM_SIGNATURE        0x57434330
#define EEPROM_VERSION          0x01

// #define DEBUG   true   // not needed, DEBUG is defined in ninja build file

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


// configuration structure
typedef struct {
    uint8_t version;
    uint8_t fans;
} config_t;

typedef struct {
    uint8_t gpio_port;
    uint8_t pwm_port;
} fan_config_t;

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

typedef struct {
    uint32_t signature;
    config_t config;
    fan_config_t fan_config[MAX_FANS];
    fan_user_config_t fan_user_config[MAX_FANS];
    fan_calibration_t fan_calibration[MAX_FANS];
} eeprom_t;

eeprom_t eeprom = {
    .signature = EEPROM_SIGNATURE,
    .config = {
        .version = EEPROM_VERSION,
        .fans = 5,
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
    }
};

config_t *config = &eeprom.config;
fan_config_t *fan_config = eeprom.fan_config;
fan_calibration_t *fan_calibration = eeprom.fan_calibration;
fan_user_config_t *fan_user_config = eeprom.fan_user_config;

typedef struct {
    bool enabled;
    bool calibrated;
    bool pwm_control;
    bool calibration_in_progress;
    uint16_t pwm;
    uint16_t rpm;
} fan_runtime_t;

fan_runtime_t fan_runtime[MAX_FANS] = {
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm = 0, .rpm = 0},
    { .enabled = true, .calibrated = false, .pwm_control = true, .calibration_in_progress = false, .pwm = 0, .rpm = 0},
};


// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

// Thermometer instance
twr_tmp112_t tmp112;
uint16_t button_click_count = 0;


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

// FAN set speed using calibration data
void fan_set_speed(uint8_t fan, float speed)
{
#ifdef DEBUG            
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
            fan_runtime[fan].pwm = fan_calibration[fan].points[FAN_CALIBRATION_STEPS-1];
        else if ((int)index < 0)
            fan_runtime[fan].pwm = fan_calibration[fan].points[0];
        else
            // linear interpolation between 2 points in calibration data (PWM values)
            fan_runtime[fan].pwm = (uint16_t)((float)fan_calibration[fan].points[(int)index] * (1.0 - weight) + (float)fan_calibration[fan].points[(int)index+1] * weight);
        // cap PWM value to minimum PWM value as detected by calibration process
        if (fan_runtime[fan].pwm < fan_calibration[fan].min_pwm)
            fan_runtime[fan].pwm = fan_calibration[fan].min_pwm;
    } else {
        fan_runtime[fan].pwm = (uint16_t)((float)FAN_PWM_MAX * speed);
        // cap PWM value to minimum PWM value according to specification
        if (fan_runtime[fan].pwm < FAN_PWM_MIN)
            fan_runtime[fan].pwm = FAN_PWM_MIN;
    }
    // cap PWM value to maximum PWM value
    if (fan_runtime[fan].pwm > FAN_PWM_MAX)
        fan_runtime[fan].pwm = FAN_PWM_MAX;
#ifdef DEBUG
    twr_log_debug("fan_set_speed(): FAN=%i, computed: speed=%.2f, PWM=%i", fan, speed, fan_runtime[fan].pwm);
#endif
    twr_pwm_set(fan_config[fan].pwm_port, fan_runtime[fan].pwm);
}

// ==== FAN calibration ====

// FAN calibration
struct fan_calibration_measure_t
{
    uint16_t pwm;
    uint16_t rpm;
};

struct fan_calibration_measure_t fan_calibration_measure[MAX_FANS][FAN_CALIBRATION_STEPS];
uint8_t fan_list[MAX_FANS] = {0};

// compute calibration data from measured values
void fan_compute_calibration(uint8_t fan)
{
    uint16_t min_rpm = 0;
    uint16_t max_rpm = 0;

    // preset default sane values according to specification
    fan_user_config[fan].max_speed = 1.0;
    fan_user_config[fan].min_speed = 0.2;
    fan_calibration[fan].min_pwm = FAN_PWM_MIN;

#ifdef DEBUG
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
#ifdef DEBUG
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
#ifdef DEBUG
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
#ifdef DEBUG
    twr_log_debug("fan_compute_calibration(): FAN %i: RPM peaks of lower 20%% and 10%% PWM range, min20=%i, max20=%i, min10=%i, max10=%i", fan, pwm20_min_rpm, pwm20_max_rpm, pwm10_min_rpm, pwm10_max_rpm);
#endif
    if ((float)(pwm20_max_rpm - pwm20_min_rpm) / (float)pwm20_max_rpm > 0.6)
    {
        if ((float)(pwm10_max_rpm - pwm10_min_rpm) / (float)pwm10_max_rpm > 0.6) {
#ifdef DEBUG            
            twr_log_debug("fan_compute_calibration(): FAN %i: lower 0-20%% of PWM range seems usable", fan);
#endif
            fan_calibration[fan].calibrated = true;
            fan_user_config[fan].min_speed = 0.0;
            fan_calibration[fan].min_pwm = 0;
        } else {
#ifdef DEBUG            
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

#ifdef DEBUG            
    float rpm_deviation[FAN_CALIBRATION_STEPS];
#endif
    // Compute calibration points from measured values
    // Note: we're computing calibration points in reverse order as we measured starting with 100% speed, but we want to have 0% speed as first calibration point
    for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++)
    {
        float speed = 1.0 - (float)i / (float)(FAN_CALIBRATION_STEPS-1);
        uint16_t rpm = (uint16_t)((float)max_rpm * speed);
#ifdef DEBUG            
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
#ifdef DEBUG            
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
#ifdef DEBUG            
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
#ifdef DEBUG            
            twr_log_debug("fan_calibration_step(): FAN %i RPM stabilization step: %i != %i", fan, rpm, last_rpm[fan]);
#endif
            last_rpm[fan] = rpm;
            twr_scheduler_plan_current_relative(FAN_CALIBRATION_TIME_RPM);
            return;
        }
        fan_calibration_measure[fan][step[fan]-1].rpm = rpm;
        last_rpm[fan] = 0;
#ifdef DEBUG            
        twr_log_debug("fan_calibration_step(): FAN %i step %d, speed=%.2f (pwm=%d), prev rpm=%i", fan, step[fan], speed, pwm, fan_calibration_measure[fan][step[fan]-1].rpm);
#endif
    } else {
#ifdef DEBUG            
        twr_log_debug("fan_calibration_step(): FAN %i step 0, speed=%.2f (pwm=%d)", fan, speed, pwm);
#endif
    }
    step[fan]++;
    if (step[fan] > FAN_CALIBRATION_STEPS) {
        // set FAN speed to previous value
        twr_pwm_set(fan_config[fan].pwm_port, fan_runtime[fan].pwm);
        fan_runtime[fan].calibration_in_progress = false;
        step[fan] = 0;
        twr_scheduler_unregister(twr_scheduler_get_current_task_id());
#ifdef DEBUG            
        twr_log_debug("fan_calibration_step(): FAN %i calibration done", fan);
        for (uint8_t i = 0; i < FAN_CALIBRATION_STEPS; i++)
            twr_log_debug("fan_calibration_step(): FAN %i measured values: pwm=%i, rpm=%i", fan, fan_calibration_measure[fan][i].pwm, fan_calibration_measure[fan][i].rpm);
#endif
        // compute calibration data from measured values
        fan_compute_calibration(fan);
        // TODO: save calibration data to EEPROM
    } else {
#ifdef DEBUG            
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
#ifdef DEBUG            
    twr_log_debug("fan_calibration_start(): FAN %i calibration start", fan);
#endif
    fan_calibration[fan].calibrated = false;
    fan_runtime[fan].calibration_in_progress = true;
    fan_list[fan] = fan;
    twr_pwm_set(fan_config[fan].pwm_port, FAN_PWM_MAX);
    twr_scheduler_register(fan_calibration_step, &fan_list[fan], twr_tick_get() + FAN_CALIBRATION_TIME_INIT);
}



#define FAN_SPEED_STEP 0.1
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
        _pwm_init(fan_config[f].pwm_port, FAN_PWM_MAX);
        fan_set_speed(f, FAN_SPEED_INIT);
        twr_pwm_enable(fan_config[f].pwm_port);
#ifdef DEBUG            
        twr_log_debug("APP: PWM init, FAN=%i, port=%i", f, fan_config[f].pwm_port);
#endif
        twr_gpio_init(fan_config[f].gpio_port);
        twr_gpio_set_mode(fan_config[f].gpio_port, TWR_GPIO_MODE_INPUT);
        twr_gpio_set_pull(fan_config[f].gpio_port, TWR_GPIO_PULL_UP);
#ifdef DEBUG            
        twr_log_debug("APP: RPM reading init, FAN=%i, port=%i", f, fan_config[f].gpio_port);
#endif
        // temporary to test calibration
        fan_calibration_start(f);
    }
    twr_system_pll_enable();

    // Initialize radio
    twr_radio_init(TWR_RADIO_MODE_NODE_SLEEPING);
    // Send radio pairing request
    twr_radio_pairing_request("water-cooler", FW_VERSION);
}

// Application task function (optional) which is called peridically if scheduled
void application_task(void)
{
    // static int counter = 0;

    // Log task run and increment counter
    // twr_log_debug("APP: Task run (count: %d)", ++counter);
    twr_log_debug("APP: RPM: %d", get_rpm(0));

    // Plan next run of this task in 1000 ms
    twr_scheduler_plan_current_from_now(60000);
}
