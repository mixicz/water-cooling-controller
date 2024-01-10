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
+- save config and calibration data to EEPROM
+- load config and calibration data from EEPROM
+- automatic FAN calibration
/- performance profiles for FANs
- MQTT - reporing temperatures, FAN speeds
- MQTT - reporting alarms (unusually high temperature, FAN failure, start/stop calibration, ...)
- MQTT - commands to set profile, start calibration, ...
- MQTT - set config values
- LED control:
    - on - NTC calibration in progress, waiting for temperature stabilization
    - brief flashes - NTC calibration in progress, waiting for temperature change
    - blinking - fan calibration in progress
    - off - normal operation
*/

#include <application.h>

// ==== EEPROM ====
eeprom_t eeprom = {
    .signature = EEPROM_SIGNATURE,
    .config = {
        .version = EEPROM_VERSION,
        .fan_ramp_up_step_time = 6000 / FAN_PWM_MAX,        // 6 seconds to ramp up from 0 to 100% speed
        .fan_ramp_down_step_time = 30000 / FAN_PWM_MAX,     // 30 seconds to ramp up from 100% to 0% speed
        .pump_ramp_up_step_time = 4000 / FAN_PWM_MAX,       // 4 seconds to ramp up from 0 to 100% speed
        .pump_ramp_down_step_time = 30000 / FAN_PWM_MAX,    // 30 seconds to ramp up from 100% to 0% speed
    },
    .fan_user_config = {
        { .min_speed = 0.2, .max_speed = 1.0, .default_speed = FAN_SPEED_INIT, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .default_speed = FAN_SPEED_INIT, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .default_speed = FAN_SPEED_INIT, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .default_speed = FAN_SPEED_INIT, .is_pump = false},
        { .min_speed = 0.2, .max_speed = 1.0, .default_speed = FAN_SPEED_INIT, .is_pump = true},     // last FAN port is preconfigured as water pump
    },
    .fan_calibration = {
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        { .present = true, .calibrated = false, .min_pwm = FAN_PWM_MIN, .pwm_capable = true, .points = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
    },
    .adc_calibration = {
        { .present = true, .calibrated = false, .offset = ADC_DEFAULT_OFFSET, .gain = ADC_DEFAULT_GAIN},
        { .present = true, .calibrated = false, .offset = ADC_DEFAULT_OFFSET, .gain = ADC_DEFAULT_GAIN},
        { .present = true, .calibrated = false, .offset = ADC_DEFAULT_OFFSET, .gain = ADC_DEFAULT_GAIN},
        { .present = true, .calibrated = false, .offset = ADC_DEFAULT_OFFSET, .gain = ADC_DEFAULT_GAIN},
        { .present = true, .calibrated = false, .offset = ADC_DEFAULT_OFFSET, .gain = ADC_DEFAULT_GAIN},
        { .present = true, .calibrated = false, .offset = ADC_DEFAULT_OFFSET, .gain = ADC_DEFAULT_GAIN},
    },
    .sensor_onewire_map = {
        {0},
    },
    .sensor_const = {
        { .temperature = 0.0},
    },
    .thermal_zone = {
        { .name = "zero", .sensors = {0xF0, 0xF0}, .absolute = true},           // 0: virtual zone with constant value 0.0 °C
        { .name = "temp_loop", .sensors = {0x23, 0xF0}, .absolute = false},     // 1: water temperature at end of loop
        { .name = "temp_outside", .sensors = {0x10, 0xF0}, .absolute = false},  // 2
        { .name = "temp_inside", .sensors = {0x11, 0xF0}, .absolute = false},   // 3
        { .name = "delta_gpu", .sensors = {0x20, 0x21}, .absolute = true},      // 4
        { .name = "delta_cpu", .sensors = {0x22, 0x23}, .absolute = true},      // 5
        { .name = "delta_case", .sensors = {0x10, 0x11}, .absolute = true},     // 6: delta between inside and outside temperature
        { .name = "delta_water", .sensors = {0x23, 0x10}, .absolute = true},    // 7: delta between water at end of loop and outside ambient temperature
    },
    .fan_group = {
        { .name = "pump", .fans = 0x10},        // 0
        { .name = "rad_all", .fans = 0x0F},     // 1
        { .name = "rad_front", .fans = 0x03},   // 2
        { .name = "rad_top", .fans = 0x04},     // 3
        { .name = "rad_back", .fans = 0x08},    // 4
    },
    .map_rule = {
        { .name = "idle_pump", .thermal_zone = 0, .fan_group = 0, .temperature = {-0.1, 0.1}, .speed = {0.2, 0.2}}, // default rule to keep pump running at minimum speed at idle
        { .name = "idle_rad", .thermal_zone = 0, .fan_group = 1, .temperature = {-0.1, 0.1}, .speed = {0.0, 0.0}}, // default rule to keep radiator fans off at idle
        { .name = "gpu_pump", .thermal_zone = 4, .fan_group = 0, .temperature = {0.2, 3.0}, .speed = {0.2, 1.0}},
        { .name = "cpu_pump", .thermal_zone = 5, .fan_group = 0, .temperature = {0.2, 3.0}, .speed = {0.2, 1.0}},
        { .name = "case_ambient", .thermal_zone = 6, .fan_group = 1, .temperature = {10.0, 20.0}, .speed = {0.2, 0.8}},
        { .name = "water_loop", .thermal_zone = 7, .fan_group = 1, .temperature = {5.0, 25.0}, .speed = {0.0, 1.0}},
    },
    .thermal_alert = {
        { .thermal_zone = 1, .temperature = 60.0, .gt = true, .text = "High water temp at EOL! t=%.1f"},
        { .thermal_zone = 3, .temperature = 50.0, .gt = true, .text = "High temp in case! t=%.1f"},
    },
};

// config_t *config = &eeprom.config;
// fan_calibration_t *fan_calibration = eeprom.fan_calibration;
// fan_user_config_t *fan_user_config = eeprom.fan_user_config;
// adc_calibration_t *adc_calibration = eeprom.adc_calibration;
// sensor_onewire_t *sensor_onewire_map = eeprom.sensor_onewire_map;
// sensor_const_t *sensor_const = eeprom.sensor_const;
// thermal_zone_t *thermal_zone = eeprom.thermal_zone;
// fan_group_t *fan_group = eeprom.fan_group;
// map_rule_t *map_rule = eeprom.map_rule;

// 1-wire thermometer runtime data
// typedef struct {
//     bool enabled;
//     float temperature;
// } ow_runtime_t;

uint64_t ow_slave_list[OW_MAX_SLAVES];
uint8_t ow_slave_count = 0;
twr_ds2484_t ds2482;
twr_onewire_t ow;
ow_index_t ow_index[SENSOR_MAX_COUNT];
bool ow_rescan = false;

// ADC runtime data
typedef struct {
    bool enabled;
    uint16_t raw;
} adc_runtime_t;

adc_runtime_t adc_runtime[ADC_CHANNEL_COUNT];

sensor_runtime_t sensor_runtime;

// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

// Thermometer instance
twr_tmp112_t tmp112;
uint16_t button_click_count = 0;

// ==== control logic ====
// read temperature from sensor with given ID
// when DEBUG_CONTROL is defined, this function emulates changing temperature on SENSOR_BUS_CONST idx >= 8:
// - temperature oscilates between 0.0 and eeprom.sensor_const[idx].temperature,
// - period is computed as 5 seconds * 2 ^ (idx - 8),
// - timing source is twr_tick_get() which is incremented every 1 ms
float read_temperature(uint8_t id)
{
    uint8_t bus = id & 0xF0;
    uint8_t idx = id & 0x0F;
    switch (bus)
    {
        case SENSOR_BUS_I2C:
            return sensor_runtime.temp_i2c[idx];
        case SENSOR_BUS_ONEWIRE:
            return sensor_runtime.temp_onewire[idx];
        case SENSOR_BUS_ADC:
            return sensor_runtime.temp_adc[idx];
#ifdef DEBUG_CONTROL
        case SENSOR_BUS_CONST:
            {
                twr_tick_t ref = twr_tick_get();
                float period = 5000.0 * (1 << (idx - 8));
                float step = (ref % (period * 2)) / period;
                if (step > 1.0)
                    step = 2.0 - step;
                temperature = step * eeprom.sensor_const[idx].temperature;
                return temperature;
            }
#else
        case SENSOR_BUS_CONST:
            return eeprom.sensor_const[idx].temperature;
#endif
        default:
            return NAN;
    }
    return NAN;
}

// compute temperature for thermal zone
float compute_thermal_zone_temperature(uint8_t zone)
{
    float temperature = NAN;
    float t1 = read_temperature(eeprom.thermal_zone[zone].sensors[0]);
    float t2 = read_temperature(eeprom.thermal_zone[zone].sensors[1]);
    if (!isnan(t1) && !isnan(t2)) {
        temperature = t1 - t2;
        if (eeprom.thermal_zone[zone].absolute && temperature < 0)
            temperature = -temperature;
    }
#ifdef DEBUG_CONTROL
    twr_log_debug("compute_thermal_zone_temperature(): zone %i, t1=%.2f, t2=%.2f, temperature=%.2f", zone, t1, t2, temperature);
#endif    
    return temperature;
}

// process all map rules
void process_map_rules(void)
{
    float target_rpm[MAX_FANS];
    for (uint8_t i = 0; i < MAX_FANS; i++)
        target_rpm[i] = -1.0;
    for (uint8_t i = 0; i < MAX_MAP_RULES; i++)
    {
        if (eeprom.map_rule[i].thermal_zone == 0)
            continue;
        float temperature = compute_thermal_zone_temperature(eeprom.map_rule[i].thermal_zone);
        if (isnan(temperature))
            continue;
        if (temperature >= eeprom.map_rule[i].temperature[0] && temperature <= eeprom.map_rule[i].temperature[1])
        {
            // rule matches, compute target RPM using linear interpolation
            float speed = (temperature - eeprom.map_rule[i].temperature[0]) / (eeprom.map_rule[i].temperature[1] - eeprom.map_rule[i].temperature[0]);
            float rpm = eeprom.map_rule[i].speed[0] + speed * (eeprom.map_rule[i].speed[1] - eeprom.map_rule[i].speed[0]);
            if (rpm > target_rpm[eeprom.map_rule[i].fan_group])
                target_rpm[eeprom.map_rule[i].fan_group] = rpm;
#ifdef DEBUG_CONTROL
            twr_log_debug("process_map_rules(): rule %i matches, zone[%i] temperature=%.2f, rule temperature=%.1f-%.1f, rule speed=%.2f=%.2f, rpm=%.2f, target=%.2f", 
                            i, map_rule[i].thermal_zone, temperature, map_rule[i].temperature[0], map_rule[i].temperature[1], map_rule[i].speed[0], map_rule[i].speed[1], rpm, target_rpm[map_rule[i].fan_group]);
#endif                
        }
    }

    // set target RPM for all fans
    for (uint8_t i = 0; i < MAX_FANS; i++)
    {
        if (target_rpm[i] >= 0.0)
            fan_set_speed(i, target_rpm[i]);
        else
            fan_set_speed(i, eeprom.fan_user_config[i].default_speed);
    }
}

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
        twr_log_error("ow_start_conversion(): 1-wire not responding");
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
        twr_log_error("ow_read_temperature(%016llx): 1-wire not responding", *device_id);
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

int ow_runtime_idx(uint8_t list_idx) {
    for (uint8_t i = 0; i < SENSOR_MAX_COUNT; i++) {
        if (ow_index[i].idx_list == list_idx)
            return ow_index[i].idx_runtime;
    }
    return -1;
}

// read all temperatures from 1-wire devices
void ow_read_temperatures()
{
    for (uint8_t i = 0; i < ow_slave_count; i++) {
        int idx = ow_runtime_idx(i);
        if (idx >= 0)
            sensor_runtime.temp_onewire[idx] = ow_read_temperature(&ow_slave_list[i]);
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
    // initialize 1-wire bus on first call only, further calls will just rescann the bus
    static bool initialized = false;
    if (!initialized) {
        twr_ds2484_init(&ds2482, TWR_I2C_I2C0);
        twr_ds2484_enable(&ds2482);
        twr_onewire_ds2484_init(&ow, &ds2482);
        memset(ow_index, 0, sizeof(ow_index));
    }
    // enumerate all slaves on 1-wire bus
#ifdef DEBUG_ONEWIRE
    twr_log_debug("ow_init(): 1-wire bus device scan started");
#endif
    bool new_device = false;
    ow_slave_count = twr_onewire_search_all(&ow, ow_slave_list, sizeof(ow_slave_list));
    for (uint8_t i = 0; i < ow_slave_count; i++)
    {
        bool found = false;
        for (uint8_t j = 0; j < sizeof(eeprom.sensor_onewire_map) / sizeof(eeprom.sensor_onewire_map[j]); j++)
        {
            if (eeprom.sensor_onewire_map[j].address == ow_slave_list[i])
            {
                ow_index[j].idx_runtime = j;
                ow_index[j].idx_list = i;
                ow_index[j].enabled = true;
                sensor_runtime.temp_onewire[j] = NAN;
                found = true;
                twr_log_info("ow_init(): 1-wire bus scan found known device %i on position %i: %016llx", i, j, ow_slave_list[i]);
                break;
            }
        }
        if (!found)
        {
            // create new entry in sensor_onewire_map
            for (uint8_t j = 0; j < sizeof(eeprom.sensor_onewire_map) / sizeof(eeprom.sensor_onewire_map[j]); j++)
            {
                if (eeprom.sensor_onewire_map[j].address == 0)
                {
                    eeprom.sensor_onewire_map[j].address = ow_slave_list[i];
                    ow_index[j].idx_runtime = j;
                    ow_index[j].idx_list = i;
                    ow_index[j].enabled = true;
                    sensor_runtime.temp_onewire[j] = NAN;
                    found = true;
                    new_device = true;
                    twr_log_info("ow_init(): 1-wire bus scan found new device %i, storing it to unused position %i: %016llx", i, j, ow_slave_list[i]);
                    break;
                }
            }
        }
        if (!found)
        {
            twr_log_error("ow_init(): 1-wire bus scan found new device %i, but there is no free position in sensor_onewire_map", i);
        }
    }
    if (new_device)
    {
        // save new device to EEPROM
        eeprom_write();
    }
    ow_rescan = false;

    if (!initialized)
    {
        // start periodic task to read temperatures from 1-wire devices
        twr_scheduler_register(ow_read_temperatures_task, NULL, twr_tick_get() + 1000);
        initialized = true;
    }
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
            // convert ADC value to temperature - check for sane calibration values
            // if (eeprom.adc_calibration[channel].calibrated)
            if (fabs(eeprom.adc_calibration[channel].offset) > 1.0)
                temperature = (float)value * eeprom.adc_calibration[channel].gain + eeprom.adc_calibration[channel].offset;
            else
                temperature = NAN;
            // temporary for debugging
            // twr_adc_async_get_voltage(channel, &temperature);
            adc_runtime[channel].raw = value;
            // adc_runtime[channel].temperature = temperature;
            sensor_runtime.temp_adc[channel] = temperature;
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
        if (eeprom.adc_calibration[i].present)
        {
            adc_runtime[i].enabled = true;
            adc_runtime[i].raw = 0;
            sensor_runtime.temp_adc[i] = NAN;
        }
    }
    twr_adc_init();
    for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        if (eeprom.adc_calibration[i].present)
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
#define ADC_CALIBRATION_POINTS 2
// sane values from real measurements
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
twr_scheduler_task_id_t adc_calibration_task_id = 0;

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
            eeprom.adc_calibration[i].present = false;
            eeprom.adc_calibration[i].calibrated = false;
            adc_runtime[i].enabled = false;
            continue;
        }

        // compute gain
        eeprom.adc_calibration[i].gain = (adc_calibration_measure_point[1].avg_temperature - adc_calibration_measure_point[0].avg_temperature)
                                    / (float)(adc_calibration_measure_point[1].adc_value[i] - adc_calibration_measure_point[0].adc_value[i]);
        // compute offset
        eeprom.adc_calibration[i].offset = adc_calibration_measure_point[0].avg_temperature - (float)adc_calibration_measure_point[0].adc_value[i] * eeprom.adc_calibration[i].gain;
        eeprom.adc_calibration[i].present = true;
        eeprom.adc_calibration[i].calibrated = true;
        adc_runtime[i].enabled = true;
#ifdef DEBUG_ADC_CALIBRATION
        twr_log_debug("adc_compute_calibration(): ADC channel %i, gain=%.5f, offset=%.2f", i, eeprom.adc_calibration[i].gain, eeprom.adc_calibration[i].offset);
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
        int idx = ow_runtime_idx(i);
        if (idx >= 0 && ow_index[idx].enabled && !isnan(sensor_runtime.temp_onewire[idx]))
        {
            avg_temperature += sensor_runtime.temp_onewire[idx];
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
                twr_log_debug("adc_calibration_callback(): ADC %i calibration point %i: Temperature=%.2f, ADC Value=%u", j, adc_calibration_measure_point_pos, avg.avg_temperature, avg.adc_value[j]);
            }
#endif                
            // store calibration point
            adc_calibration_measure_point[adc_calibration_measure_point_pos] = avg;
            adc_last_calibration_temperature = avg.avg_temperature;
            if (++adc_calibration_measure_point_pos >= ADC_CALIBRATION_POINTS)
            {
                adc_compute_calibration();
                adc_calibration_task_id = 0;
                twr_scheduler_unregister(adc_calibration_task_id);
#ifdef DEBUG_ADC_CALIBRATION
                twr_log_debug("adc_calibration_callback(): writing calibration data to eeprom");
#endif
                eeprom_write();
#ifdef DEBUG_ADC_CALIBRATION
                twr_log_debug("adc_calibration_callback(): eeprom write done");
#endif
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
    adc_calibration_task_id = twr_scheduler_register(adc_calibration_callback, NULL, twr_tick_get() + ADC_CALIBRATION_STEP_INTERVAL);
}

// stop ADC calibration
void adc_calibration_stop(void)
{
    if (adc_calibration_task_id > 0)
        twr_scheduler_unregister(adc_calibration_task_id);
    adc_calibration_task_id = 0;
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


// Button event callback
void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    // Log button event
    twr_log_info("APP: Button event: %i", event);

    // Check event source
    if (event == TWR_BUTTON_EVENT_HOLD) {
        // flash LED to indicate long press
        twr_led_pulse(&led, 100);
        if (adc_calibration_task_id > 0) {
            // long press during NTC calibration - stop NTC calibration
            adc_calibration_stop();
        } else {
            // long press - start NTC calibration
            adc_calibration_start();
        }
    } else if (event == TWR_BUTTON_EVENT_CLICK) {
        if (adc_calibration_task_id > 0) {
            // short press during NTC calibration - stop NTC calibration and start FAN calibration
            adc_calibration_stop();
            for (uint8_t f = 0; f < MAX_FANS; f++)
            {
                if (!fan_runtime[f].calibration_in_progress)
                    fan_calibration_start(f);
            }
        } else {
            // short press during normal operation - rescan 1-wire bus
            ow_init();
        }
    }
}

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *event_param)
{
    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        float celsius;
        // Read temperature
        twr_tmp112_get_temperature_celsius(self, &celsius);

// #ifdef DEBUG            
//         twr_log_debug("tmp112_event_handler(): temperature: %.2f °C", celsius);
// #endif
        sensor_runtime.temp_i2c[0] = celsius;

        // Publish temperature on radio
        // twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &celsius);
    }
}

// Application initialization function which is called once after boot
void application_init(void)
{
    // initialize adc_runtime
    memset(&adc_runtime, 0, sizeof(adc_runtime));
    // initialize sensor_runtime with NaN values, which can be done as filling the structure with 0xFF bytes, see https://en.wikipedia.org/wiki/IEEE_754-1985#NaN
    memset(&sensor_runtime, 0xFF, sizeof(sensor_runtime));

    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);
#ifdef DEBUG
    twr_log_debug("init(): eeprom size: %i", sizeof(eeprom));
#endif

    // Initialize EEPROM
    if (!eeprom_read()) {
        // write default values to EEPROM
        eeprom_write();
    }

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
    for (uint8_t f = 0; f < MAX_FANS; f++)
    {
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("init(): PWM init, FAN=%i, port=%i", f, fan_config[f].pwm_port);
#endif
        // fan_list[f] = f;
        _pwm_init(fan_config[f].pwm_port, FAN_PWM_MAX);
        twr_pwm_enable(fan_config[f].pwm_port);
        twr_gpio_init(fan_config[f].gpio_port);
        twr_gpio_set_mode(fan_config[f].gpio_port, TWR_GPIO_MODE_INPUT);
        twr_gpio_set_pull(fan_config[f].gpio_port, TWR_GPIO_PULL_UP);
        fan_set_speed(f, FAN_SPEED_INIT);
#ifdef DEBUG_FAN_CALIBRATION
        twr_log_debug("init(): RPM reading init, FAN=%i, port=%i", f, fan_config[f].gpio_port);
        // temporary to test calibration
#endif
        if (!eeprom.fan_calibration[f].calibrated) {
            fan_calibration_start(f);
        }
    }
    // enable FAN RPM reading via GPIO (PLL is called every 1 ms)
    twr_system_pll_enable();

    // Initialize 1-wire bus
    ow_init();
    ow_start_conversion();

    // Initialize ADC
    adc_init();
#ifdef DEBUG_ADC_CALIBRATION
    // adc_calibration_start();
#endif

    // Initialize PCA9685 PWM controller
    // pca9685_init();

    mqtt_init();
}

// Application task function (optional) which is called peridically if scheduled
void application_task(void)
{
    static int counter = 0;
    counter++;

    // Log task run and increment counter
    // twr_log_debug("APP: Task run (count: %d)", ++counter);
    // twr_log_debug("APP: RPM: %d", get_rpm(1));
    float tmin = NAN;
    float tmax = NAN;
    float tavg = 0.0;
    uint8_t ow_count = 0;
    for (uint8_t t = 0; t < SENSOR_MAX_COUNT; t++)
    {
        if (ow_index[t].enabled) {
            if (isnan(tmin) || sensor_runtime.temp_onewire[ow_index[t].idx_runtime] < tmin)
                tmin = sensor_runtime.temp_onewire[ow_index[t].idx_runtime];
            if (isnan(tmax) || sensor_runtime.temp_onewire[ow_index[t].idx_runtime] > tmax)
                tmax = sensor_runtime.temp_onewire[ow_index[t].idx_runtime];
            tavg += sensor_runtime.temp_onewire[ow_index[t].idx_runtime];
            ow_count++;
        }
    }
    tavg /= (float)ow_slave_count;
    twr_log_info("APP: 1-wire thermometers: %i devices (%i active), min=%.2f, max=%.2f, avg=%.2f, delta=%.2f", ow_slave_count, ow_count, tmin, tmax, tavg, tmax-tmin);
    // print out whole sensor_runtime structure with each sensor group on separate line
    /*
    typedef struct {
        float temp_i2c[1];
        float temp_onewire[OW_MAX_SLAVES];
        float temp_adc[ADC_CHANNEL_COUNT];
    } sensor_runtime_t;
    */
#ifdef DEBUG
    twr_log_info("APP: sensor_runtime: temp_i2c: [%.2f]", sensor_runtime.temp_i2c[0]);
    twr_log_info("APP: sensor_runtime: temp_onewire: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                    sensor_runtime.temp_onewire[0], sensor_runtime.temp_onewire[1], sensor_runtime.temp_onewire[2], sensor_runtime.temp_onewire[3],
                    sensor_runtime.temp_onewire[4], sensor_runtime.temp_onewire[5], sensor_runtime.temp_onewire[6], sensor_runtime.temp_onewire[7]);
    twr_log_info("APP: sensor_runtime: temp_adc: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    sensor_runtime.temp_adc[0], sensor_runtime.temp_adc[1], sensor_runtime.temp_adc[2], sensor_runtime.temp_adc[3],
                    sensor_runtime.temp_adc[4], sensor_runtime.temp_adc[5]);
#endif
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
    twr_adc_async_measure(TWR_ADC_CHANNEL_A0);  // TODO - place this to some periodic task, it will measure all ADC channels starting with A0
    // pca9685_set_pwm(PCA9685_CHANNEL_ALL, ((counter+2)*256)&0x0FFF);
// #ifdef DEBUG_PCA9685
//     uint8_t tmp[4] = {0, 0, 0, 0};
//     pca9685_read(0x06, tmp, 4);
//     twr_log_debug("APP: PCA9685: regs: %02x %02x %02x %02x", tmp[0], tmp[1], tmp[2], tmp[3]);
// #endif

    // control logic
    process_map_rules();
    publish_temperatures();
    
    // rescan 1-wire bus if requested
    if (ow_rescan) {
        ow_rescan = false;
        ow_init();
    }

    // Plan next run of this task in 1000 ms
    twr_scheduler_plan_current_from_now(10000);
}
