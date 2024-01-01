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

// 1-wire configuration
# define OW_MAX_SLAVES 8

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

// ==== FAN RPM reading ====
// this is called every 1 ms by SDK
void HAL_SYSTICK_Callback(void)
{
    fan_HAL_SYSTICK_Callback();
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
    twr_ds2484_enable(&ds2482);
    twr_onewire_ds2484_init(&ow, &ds2482);
    // enumerate all slaves on 1-wire bus
#ifdef DEBUG_ONEWIRE
    twr_log_debug("ow_init(): 1-wire bus device scan started");
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


// ==== PCA9685 PWM controller ====
#define PCA9685_CHANNEL_ALL 61

// write to PCA9685 register
void pca9685_write(uint8_t reg, uint8_t *value, uint8_t len) {
    twr_i2c_memory_transfer_t transfer;

    transfer.device_address = 0x40;
    transfer.memory_address = reg;
    transfer.buffer = value;
    transfer.length = len;

    twr_i2c_memory_write(TWR_I2C_I2C0, &transfer);
}

// read from PCA9685 register
void pca9685_read(uint8_t reg, uint8_t *value, uint8_t len) {
    twr_i2c_memory_transfer_t transfer;

    transfer.device_address = 0x40;
    transfer.memory_address = reg;
    transfer.buffer = value;
    transfer.length = len;

    twr_i2c_memory_read(TWR_I2C_I2C0, &transfer);
}

// set PWM value for PCA9685 channel (12-bit value)
void pca9685_set_pwm(uint8_t channel, uint16_t value) {
    uint8_t data[5];

    data[0] = 0x06 + 4 * channel;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = value & 0xFF;
    data[4] = value >> 8;

    pca9685_write(0x00, data, 5);
#ifdef DEBUG_PCA9685
    twr_log_debug("pca9685_set_pwm(): channel=%i, value=%i (raw=%02x %02x)", channel, value, data[3], data[4]);
#endif
}

// Initialize PWM
void pca9685_init(void) {
    twr_i2c_init(TWR_I2C_I2C0, TWR_I2C_SPEED_400_KHZ);
#ifdef DEBUG_PCA9685
    uint8_t tmp[2] = {0, 0};
    pca9685_read(0x00, tmp, 2);
    twr_log_debug("pca9685_init(): pre-init: MODE1: %02x, MODE2: %02x", tmp[0], tmp[1]);
#endif
    // set MODE1 auto-increment bit
    uint8_t value = 0x20;
    pca9685_write(0x00, &value, 1);
#ifdef DEBUG_PCA9685
    // uint8_t tmp[2] = {0, 0};
    pca9685_read(0x00, tmp, 2);
    twr_log_debug("pca9685_init(): after-init: MODE1: %02x, MODE2: %02x", tmp[0], tmp[1]);
#endif
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
        // fan_list[f] = f;
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
    // adc_init();
    // adc_calibration_start();

    // Initialize PCA9685 PWM controller
    // pca9685_init();

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
    // uint8_t adc_count = 0;
    // tmin = NAN;
    // tmax = NAN;
    // tavg = 0.0;
    // for (uint8_t t = 0; t < ADC_CHANNEL_COUNT; t++)
    // {
    //     if (adc_runtime[t].enabled && adc_calibration[t].calibrated) {
    //         twr_log_debug("APP: ADC thermometer %i: %.2f (raw=%i)", t, adc_runtime[t].temperature, adc_runtime[t].raw);
    //         if (isnan(tmin) || adc_runtime[t].temperature < tmin)
    //             tmin = adc_runtime[t].temperature;
    //         if (isnan(tmax) || adc_runtime[t].temperature > tmax)
    //             tmax = adc_runtime[t].temperature;
    //         tavg += adc_runtime[t].temperature;
    //         adc_count++;
    //     }
    // }
    // if (adc_count > 0) {
    //     tavg /= (float)adc_count;
    //     twr_log_debug("APP: ADC thermometers: %i devices, min=%.2f, max=%.2f, avg=%.2f, delta=%.2f", adc_count, tmin, tmax, tavg, tmax-tmin);
    // }
    // for (uint8_t t = 0; t < ADC_CHANNEL_COUNT; t++)
    // {
    //     if (adc_runtime[t].enabled) {
    //         twr_log_debug("APP: ADC thermometer %i: %.2f", t, adc_runtime[t].temperature);
    //     }
    // }
    // twr_adc_async_measure(TWR_ADC_CHANNEL_A0);  // TODO - place this to some periodic task, it will measure all ADC channels starting with A0
    // pca9685_set_pwm(PCA9685_CHANNEL_ALL, ((counter+2)*256)&0x0FFF);
// #ifdef DEBUG_PCA9685
//     uint8_t tmp[4] = {0, 0, 0, 0};
//     pca9685_read(0x06, tmp, 4);
//     twr_log_debug("APP: PCA9685: regs: %02x %02x %02x %02x", tmp[0], tmp[1], tmp[2], tmp[3]);
// #endif

    // Plan next run of this task in 1000 ms
    twr_scheduler_plan_current_from_now(10000);
}
