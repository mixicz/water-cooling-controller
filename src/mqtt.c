#include <mqtt.h>
#include <application.h>


// ==== MQTT ====

/*
typedef enum
{
BC_RADIO_SUB_PT_BOOL = 0,
BC_RADIO_SUB_PT_INT = 1,
BC_RADIO_SUB_PT_FLOAT = 2,
BC_RADIO_SUB_PT_STRING = 3,
BC_RADIO_SUB_PT_NULL = 4,

} bc_radio_sub_pt_t;
 */

static char mqtt_bufer[256];
// subscribe table, format: topic, expect payload type, callback, user param
static const bc_radio_sub_t mqtt_subs[] = {
    // state/set
    //     {"led-pwm/-/config/set", BC_RADIO_SUB_PT_STRING, led_config_set, NULL },
    {"water-cooler/-/config/set", BC_RADIO_SUB_PT_STRING, config_set, mqtt_bufer},
    {"water-cooler/-/config/get", BC_RADIO_SUB_PT_STRING, config_get, mqtt_bufer},
};

// MQTT callbacks
/*
###common elements
- sensor ID is 8-bit (2 digit) hexadecimal number, where upper 4 bits represent sensor bus and lower 4 bits represent sensor index on that bus,
    - sensor bus is one of:
        - `0` - I2C (only one sensor supported with ID 0x00)
        - `1` - 1-wire (up to 8 sensors supported with IDs 0x10..0x17)
        - `2` - ADC (up to 6 sensors supported with IDs 0x20..0x25)
        - `F` - constant value (up to 16 values supported with IDs 0xF0..0xFF, 0xF0 is reserved for 0°C)
    - example:
        - `0x10` - 1-wire sensor with index 0
        - `0xF0` - constant of 0°C
- temperature is float in °C
- speed is float in <0..1> interval
- name is string, max 15 chars

### Sensor onewire
- maps 1-wire sensor address to sensor ID
- string representation as `O:<index>=<address>`, where
    - *index* - single hex digit (`0`..`F`) as 1-wire sensor index,
    - *address* - 64-bit hexadecimal number as 1-wire device address

### Thermal zone
- represents thermal zone as difference between 2 sensors (constant value sensor can be used to represent real temperature)
- string interface representation as `Z:<index>=<ID0>[-|]<ID2>#<name>`, where:
    - index is single hexadecimal number,
    - ID0 and ID1 are 2-digit hexadecimal numbers representing sensor IDs,
    - `[-|]` indicates if resulting temperature is 
        - `-` - subtraction of temperatures from ID0 and ID1, temp = ID0 - ID1 (may be negative)
        - `|` - difference between temperatures from ID0 and ID1, temp = abs(ID0 - ID1) (always positive)
- example:
    - `Z:0=00-F0#controller ambient` - represents ambient temperature around controller defined as difference between onboard sensor and fixed value of 0°C,
    - `Z:1=10|11#inside case delta` - delta between 1-wire sensors `0` and `1`, for example difference between outside ambient temperature and temperature inside PC case,

### Fan group
- string representation as `G:<index>=<binary value>#<name>`, where
    - each bit in 5-bit binary value represent PWM ports, with `1` meaning the PWM output is part of the group and leftmost bit has highest index,
- example:
    - `G:0=10000#pump` - only PWM port 5 (pump) is part of the group,
    - `G:1=01111#all radiators` - PWM ports 1 to 4 are part of the group
    - `G:2=00011#front radiator` - PWM ports 1 and 2 are part of the group

### Map rule 
- string representation as `R:<index>=Z<zone>(t1,t2)G<group>(s1,s2)#<name>`, where:
    - *index* - single hex digit (`0`..`F`) as map rule index,
    - *zone* - single hex digit (`0`..`F`) as thermal zone index,
    - *group* - single hex digit (`0`..`F`) as fan group index,
    - *t1* / *t2* - start/end of temperature range as float in °C,
    - *s1* / *s2* - start/end of relative speed range as float in <0..1> interval,
    - *name* - string, max 15 chars.
- example:
    - `R:0=Z4(0.2,3)G1(0.0,1)#GPU delta to AllR`

### Thermal alert definition
- string representation as `A:<index>=Z<zone>[operator]temp#<text>`, where:
    - *index* - single hex digit (`0`..`F`) as map rule index,
    - *zone* - single hex digit (`0`..`F`) as thermal zone index,
    - *operator* - ether `>` or `<`, indicating whether zone temperature has to be higher (`>`) or lower (`<`) than specified temperature,
    - *temp* - temperature as float in °C,
    - *text* - string, max 31 chars, will be used as format specifier in snprintf() with single float parameter representing thermal zone temperature.
- example:
    - `A:0=Z2>50#Temperature in cooling loop is too high! t=%.1f`

*/

int hex2int(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

void mqtt_error(const char *message, const char *topic, const char *payload) {
    char pub[60];
    twr_log_error("mqtt_error(): ERROR: %s, topic: '%s', payload: '%s'", message, topic, payload);
    snprintf(pub, sizeof(pub), "ERROR: %s", message);
    bc_radio_pub_string(topic, pub);
}

void mqtt_reply(const char *message, const char *topic, const char *payload) {
    char pub[60];
#ifdef DEBUG_MQTT
    twr_log_debug("mqtt_reply(): reply: %s, topic: '%s', original payload: '%s'", message, topic, payload);
#endif
    snprintf(pub, sizeof(pub), "ERROR: %s", message);
    bc_radio_pub_string(topic, pub);
}

bool config_set_sensor_onewire(const char * cfg, const char *topic) {
    uint8_t index = hex2int(cfg[2]);
    if (index < 0) {
        mqtt_error("invalid config index", topic, cfg);
        return false;
    }

    uint64_t address = 0;
    for (int i = 4; i < 20; i++) {
        int digit = hex2int(cfg[i]);
        if (digit < 0) {
            mqtt_error("invalid config address", topic, cfg);
            return false;
        }
        address = (address << 4) | digit;
    }

    // find if there is already sensor with given index
    for (uint8_t i = 0; i < OW_MAX_SLAVES; i++) {
        if (eeprom.sensor_onewire_map[i].index == index) {
            eeprom.sensor_onewire_map[i].address = address;
            // in case of address == 0, sensor entry is deleted
            if (address == 0)
                eeprom.sensor_onewire_map[i].index = 0;
            // set flag to rescan 1-wire bus after delay
            ow_rescan = true;
            return true;
        }
    }

    // if we try to delete non-existing sensor, return true
    if (address == 0)
        return true;

    // find first free slot in eeprom.sensor_onewire_map[i]
    for (uint8_t i = 0; i < OW_MAX_SLAVES; i++) {
        if (eeprom.sensor_onewire_map[i].address == 0) {
            eeprom.sensor_onewire_map[i].index = index;
            eeprom.sensor_onewire_map[i].address = address;
            // set flag to rescan 1-wire bus after delay
            ow_rescan = true;
            return true;
        }
    }

    mqtt_error("no free slots for 1-wire sensor", topic, cfg);
    return false;
}

bool config_set_thermal_zone(const char * cfg, const char *topic) {
    // TODO
    return false;
}

bool config_set_fan_group(const char * cfg, const char *topic) {
    // TODO
    return false;
}

bool config_set_map_rule(const char * cfg, const char *topic) {
    // TODO
    return false;
}

void config_set(uint64_t *id, const char *topic, void *value, void *param) {
    char *cfg = (char *)value;
#ifdef DEBUG_MQTT
    twr_log_debug("config_set(): MQTT topic: '%s', payload: '%s'", topic, cfg);
#endif

    if (cfg == NULL) {
        mqtt_error("NULL config", topic, cfg);
        return;
    }

    if (cfg[1] != ':') {
        mqtt_error("invalid config format", topic, cfg);
        return;
    }

    bool ok = false;
    switch (cfg[0]) {
        case 'O':
            ok = config_set_sensor_onewire(cfg, topic);
            break;
        case 'Z':
            ok = config_set_thermal_zone(cfg, topic);
            break;
        case 'G':
            ok = config_set_fan_group(cfg, topic);
            break;
        case 'R':
            ok = config_set_map_rule(cfg, topic);
            break;
        default:
            mqtt_error("invalid config type", topic, cfg);
            return;
    }

    if (ok) {
        mqtt_reply("OK", topic, cfg);
        eeprom_write();
    }
}

// Dumps configuration as individual MQTT messages in same format as expected by config_set().
void config_get(uint64_t *id, const char *topic, void *value, void *param) {
    // TODO
}


// MQTT publish topics
void publish_temperatures(void) {
    static sensor_runtime_t last_sensor_runtime = {0};
    static twr_tick_t last_publish_i2c[sizeof(sensor_runtime.temp_i2c) / sizeof(sensor_runtime.temp_i2c[0])] = {0};
    static twr_tick_t last_publish_onewire[sizeof(sensor_runtime.temp_onewire) / sizeof(sensor_runtime.temp_onewire[0])] = {0};
    static twr_tick_t last_publish_adc[sizeof(sensor_runtime.temp_adc) / sizeof(sensor_runtime.temp_adc[0])] = {0};
    char topic_buff[32];
    // sprintf(topic_buff, "temperature/%d/off", led_id);
    // bc_radio_pub_float(topic_buff, pub_json);

    // publish temperatures from I2C sensors
    for (uint8_t i = 0 ; i < sizeof(sensor_runtime.temp_i2c) / sizeof(sensor_runtime.temp_i2c[0]); i++)
    {
        if (isnan(sensor_runtime.temp_i2c[i]))
            continue;
        if (fabs(sensor_runtime.temp_i2c[i] - last_sensor_runtime.temp_i2c[i]) > MQTT_REPORT_TEMP_DELTA || twr_tick_get() - last_publish_i2c[i] > MQTT_REPORT_TEMP_INTERVAL)
        {
            last_publish_i2c[i] = twr_tick_get();
            last_sensor_runtime.temp_i2c[i] = sensor_runtime.temp_i2c[i];
            sprintf(topic_buff, "thermometer/%d:%d/temperature", SENSOR_BUS_I2C/16, i);
            bc_radio_pub_float(topic_buff, &sensor_runtime.temp_i2c[i]);
        }
    }

    // publish temperatures from 1-wire sensors
    for (uint8_t i = 0 ; i < sizeof(sensor_runtime.temp_onewire) / sizeof(sensor_runtime.temp_onewire[0]); i++)
    {
        if (isnan(sensor_runtime.temp_onewire[i]))
            continue;
        if (fabs(sensor_runtime.temp_onewire[i] - last_sensor_runtime.temp_onewire[i]) > MQTT_REPORT_TEMP_DELTA || twr_tick_get() - last_publish_onewire[i] > MQTT_REPORT_TEMP_INTERVAL)
        {
            last_publish_onewire[i] = twr_tick_get();
            last_sensor_runtime.temp_onewire[i] = sensor_runtime.temp_onewire[i];
            sprintf(topic_buff, "thermometer/%d:%d/temperature", SENSOR_BUS_ONEWIRE/16, ow_index[i].idx_list);
            bc_radio_pub_float(topic_buff, &sensor_runtime.temp_onewire[i]);
        }
    }

    // publish temperatures from ADC sensors
    for (uint8_t i = 0 ; i < sizeof(sensor_runtime.temp_adc) / sizeof(sensor_runtime.temp_adc[0]); i++)
    {
        if (isnan(sensor_runtime.temp_adc[i]))
            continue;
        if (fabs(sensor_runtime.temp_adc[i] - last_sensor_runtime.temp_adc[i]) > MQTT_REPORT_TEMP_DELTA || twr_tick_get() - last_publish_adc[i] > MQTT_REPORT_TEMP_INTERVAL)
        {
            last_publish_adc[i] = twr_tick_get();
            last_sensor_runtime.temp_adc[i] = sensor_runtime.temp_adc[i];
            sprintf(topic_buff, "thermometer/%d:%d/temperature", SENSOR_BUS_ADC/16, i);
            bc_radio_pub_float(topic_buff, &sensor_runtime.temp_adc[i]);
        }
    }
}


