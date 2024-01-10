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
static twr_radio_sub_t mqtt_subs[] = {
    // state/set
    //     {"led-pwm/-/config/set", BC_RADIO_SUB_PT_STRING, led_config_set, NULL },
    {"wc/-/config/set", BC_RADIO_SUB_PT_STRING, config_set, mqtt_bufer},
    {"wc/-/config/get", BC_RADIO_SUB_PT_NULL, config_get, mqtt_bufer},
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

### Sensor fixed
- maps fixed sensor value to sensor index
- string representation as `F:<index>=<temperature>`, where
    - *index* - single hex digit (`0`..`F`) as fixed sensor index,
    - *temperature* - float value in °C

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

#define hex_at(var, pos) int var = hex2int(cfg[pos]); if (var < 0 || var >= MAX_THERMAL_ZONES) { mqtt_error("invalid config " #var, topic, cfg); twr_log_error("config_set_XXX(): error parsing hex value at pos %d in string \"%s\"", pos, cfg); return false; }

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
    twr_radio_pub_string("cfg/err", pub);
}

void mqtt_reply(const char *message, const char *topic, const char *payload) {
#ifdef DEBUG_MQTT
    twr_log_debug("mqtt_reply(): reply: %s, topic: '%s', original payload: '%s'", message, topic, payload);
#endif
    twr_radio_pub_string("cfg/set", message);
}

bool config_set_sensor_onewire(const char * cfg, const char *topic) {
    hex_at(index, 2);
    if (index < 0 || index >= OW_MAX_SLAVES) {
        mqtt_error("invalid config index", topic, cfg);
        return false;
    }

    uint64_t address = 0;
    for (int i = 4; i < 20 && cfg[i]; i++) {
        int digit = hex2int(cfg[i]);
        if (digit < 0) {
            mqtt_error("invalid config address", topic, cfg);
            return false;
        }
        address = (address << 4) | digit;
    }

    eeprom.sensor_onewire_map[index].address = address;

    ow_rescan = true;
    return true;
}

// fixed sensor configuration
// F:0=20.0
bool config_set_sensor_fixed(const char * cfg, const char *topic) {
    hex_at(index, 2);
    if (index < 0 || index >= SENSOR_CONST_COUNT) {
        mqtt_error("invalid config index", topic, cfg);
        return false;
    }

    // do not allow to set value for indexes 0..3 (reserved)
    if (index <= 3) {
        mqtt_error("modifying reserved values is not allowed!", topic, cfg);
        return false;
    }

    int end = float_end(&cfg[4]);
    float temperature = strtof(&cfg[4], NULL);

    eeprom.sensor_const[index].temperature = temperature;

    return true;
}

#define tz_validate_char(pos, c) if (cfg[pos] != c) { mqtt_error("invalid thermal zone format", topic, cfg); twr_log_error("config_set_thermal_zone(): invalid character, expected '%c' instead of '%c' at position %d in config string.", c, cfg[pos], pos); return false; }

// Z:0=00-F0#controller ambient
bool config_set_thermal_zone(const char * cfg, const char *topic) {
    hex_at(index, 2);
    tz_validate_char(1, ':');
    tz_validate_char(3, '=');

    // "Z:X=" is meant to delete thermal zone at index X
    if (cfg[4] == 0) {
#ifdef DEBUG_MQTT
        twr_log_debug("config_set_thermal_zone(): deleting thermal zone at index %d", index);
#endif
        eeprom.thermal_zone[index].sensors[0] = 0;
        eeprom.thermal_zone[index].sensors[1] = 0;
        eeprom.thermal_zone[index].absolute = false;
        eeprom.thermal_zone[index].name[0] = 0;
        return true;
    }

    uint8_t sensors[2] = {0, 0};
    bool absolute = false;
    char *name = NULL;

    // parse sensor 1
    int s1 = hex2int(cfg[4]);
    int s2 = hex2int(cfg[5]);
    if (s1 < 0 || s1 >= 16 || s2 < 0 || s2 >= 16) {
        mqtt_error("invalid config sensors", topic, cfg);
        return false;
    }
    sensors[0] = s1*16+s2;

    // parse sensor 2
    s1 = hex2int(cfg[7]);
    s2 = hex2int(cfg[8]);
    if (s1 < 0 || s1 >= 16 || s2 < 0 || s2 >= 16) {
        mqtt_error("invalid config sensors", topic, cfg);
        return false;
    }
    sensors[1] = s1*16+s2;

    // parse absolute flag
    switch (cfg[6]) {
        case '-':
            absolute = false;
            break;
        case '|':
            absolute = true;
            break;
        default:
            mqtt_error("invalid config absolute flag", topic, cfg);
            return false;
    }

    // check if there is a zone name after optional #
    if (cfg[9] == '#') {
        name = (char *)&cfg[10];
        if (strlen(name) > THERMAL_ZONE_NAME_LENGTH-1) {
            // name too long - log error and cut it
            twr_log_error("config_set_thermal_zone(): thermal zone name too long, cutting to %d chars (%s)", THERMAL_ZONE_NAME_LENGTH-1, name);
            name[THERMAL_ZONE_NAME_LENGTH-1] = 0;
        }
    }

    // store thermal zone to eeprom at given index
    eeprom.thermal_zone[index].sensors[0] = sensors[0];
    eeprom.thermal_zone[index].sensors[1] = sensors[1];
    eeprom.thermal_zone[index].absolute = absolute;
    if (name != NULL)
        strncpy(eeprom.thermal_zone[index].name, name, THERMAL_ZONE_NAME_LENGTH);
    else
        eeprom.thermal_zone[index].name[0] = 0;

    return true;
}

#define fg_validate_char(pos, c) if (cfg[pos] != c) { mqtt_error("invalid fan group format", topic, cfg); twr_log_error("config_set_fan_group(): invalid character, expected '%c' instead of '%c' at position %d in config string.", c, cfg[pos], pos); return false; }

// G:0=10000#pump
bool config_set_fan_group(const char * cfg, const char *topic) {
    int index = hex2int(cfg[2]);
    if (index < 0 || index >= MAX_FAN_GROUPS) {
        mqtt_error("invalid config index", topic, cfg);
        return false;
    }
    fg_validate_char(1, ':');
    fg_validate_char(3, '=');

    // "G:X=" is meant to delete fan group at index X
    if (cfg[4] == 0) {
#ifdef DEBUG_MQTT
        twr_log_debug("config_set_fan_group(): deleting fan group at index %d", index); 
#endif
        eeprom.fan_group[index].fans = 0;
        eeprom.fan_group[index].name[0] = 0;
        return true;
    }

    uint8_t fans = 0;
    for (int i = 4; i < 9; i++) {
        if (cfg[i] == '1')
            fans |= 1 << (8 - i);
        else if (cfg[i] != '0') {
            mqtt_error("invalid fan group specified", topic, cfg);
            return false;
        }
    }

    // check if there is a group name after optional #
    char *name = NULL;
    if (cfg[9] == '#') {
        name = (char *)&cfg[10];
        if (strlen(name) > FAN_GROUP_NAME_LENGTH-1) {
            // name too long - log error and cut it
            twr_log_error("config_set_fan_group(): fan group name too long, cutting to %d chars (%s)", FAN_GROUP_NAME_LENGTH-1, name);
            name[FAN_GROUP_NAME_LENGTH-1] = 0;
        }
    }

    // store fan group to eeprom at given index
    eeprom.fan_group[index].fans = fans;
    if (name != NULL)
        strncpy(eeprom.fan_group[index].name, name, FAN_GROUP_NAME_LENGTH);
    else
        eeprom.fan_group[index].name[0] = 0;
    
    return true;
}

#define mr_validate_char(pos, c) if (cfg[pos] != c) { mqtt_error("invalid map rule format", topic, cfg); twr_log_error("config_set_map_rule(): invalid character, expected '%c' instead of '%c' at position %d in config string.", c, cfg[pos], pos); return false; }

int float_end(char *str) {
    int i = 0;
    while (str[i] && (str[i] == '.' || (str[i] >= '0' && str[i] <= '9')))
        i++;
    return i;
}

// `R:0=Z4(0.2,3)G1(0.0,1)#GPU delta to AllR`
// cannot use regex because of memory constraints
bool config_set_map_rule(char * cfg, const char *topic) {
    hex_at(index, 2);
    mr_validate_char(1, ':');
    mr_validate_char(3, '=');

    // "R:X=" is meant to delete map rule at index X
    if (cfg[4] == 0) {
#ifdef DEBUG_MQTT
        twr_log_debug("config_set_map_rule(): deleting map rule at index %d", index);
#endif
        eeprom.map_rule[index].thermal_zone = 0;
        eeprom.map_rule[index].fan_group = 0;
        eeprom.map_rule[index].temperature[0] = 0;
        eeprom.map_rule[index].temperature[1] = 0;
        eeprom.map_rule[index].speed[0] = 0;
        eeprom.map_rule[index].speed[1] = 0;
        eeprom.map_rule[index].name[0] = 0;
        return true;
    }

    // parse thermal zone
    mr_validate_char(4, 'Z');
    hex_at(zone_index, 5);
    mr_validate_char(6, '(');
    int end = float_end(&cfg[7]);
    mr_validate_char(7+end, ',');
    cfg[7+end] = 0;
    float zone_temp1 = strtof(&cfg[7], NULL);
    int pos = 8+end;
    end = float_end(&cfg[pos]);
    mr_validate_char(pos+end, ')');
    cfg[pos+end] = 0;
    float zone_temp2 = strtof(&cfg[pos], NULL);
    pos += end+1;
    mr_validate_char(pos, 'G');
    hex_at(group_index, pos+1);
    mr_validate_char(pos+2, '(');
    end = float_end(&cfg[pos+3]);
    mr_validate_char(pos+3+end, ',');
    cfg[pos+3+end] = 0;
    float group_speed1 = strtof(&cfg[pos+3], NULL);
    pos += 4+end;
    end = float_end(&cfg[pos]);
    mr_validate_char(pos+end, ')');
    cfg[pos+end] = 0;
    float group_speed2 = strtof(&cfg[pos], NULL);
    pos += end+1;
    char *name = NULL;
    if (cfg[pos] == '#') {
        name = &cfg[pos+1];
        if (strlen(name) > MAP_RULE_NAME_LENGTH-1) {
            // name too long - log error and cut it
            twr_log_error("config_set_map_rule(): map rule name too long, cutting to %d chars (%s)", MAP_RULE_NAME_LENGTH-1, name);
            name[MAP_RULE_NAME_LENGTH-1] = 0;
        }
    }

    // store map rule to eeprom at given index
    eeprom.map_rule[index].thermal_zone = zone_index;
    eeprom.map_rule[index].fan_group = group_index;
    eeprom.map_rule[index].temperature[0] = zone_temp1;
    eeprom.map_rule[index].temperature[1] = zone_temp2;
    eeprom.map_rule[index].speed[0] = group_speed1;
    eeprom.map_rule[index].speed[1] = group_speed2;
    if (name != NULL)
        strncpy(eeprom.map_rule[index].name, name, MAP_RULE_NAME_LENGTH);
    else
        eeprom.map_rule[index].name[0] = 0;

#ifdef DEBUG_MQTT
    twr_log_debug("config_set_map_rule(): parsed map rule: index=%d, zone=%d, temp1=%.1f, temp2=%.1f, group=%d, speed1=%.1f, speed2=%.1f, name=%s", index, zone_index, zone_temp1, zone_temp2, group_index, group_speed1, group_speed2, name);
#endif

    return true;
}

#define ta_validate_char(pos, c) if (cfg[pos] != c) { mqtt_error("invalid thermal alert format", topic, cfg); twr_log_error("config_set_thermal_alert(): invalid character, expected '%c' instead of '%c' at position %d in config string.", c, cfg[pos], pos); return false; }

// A:0=Z2>50#Temperature in cooling loop is too high! t=%.1f
bool config_set_thermal_alert(char * cfg, const char *topic) {
    hex_at(index, 2);
    ta_validate_char(1, ':');
    ta_validate_char(3, '=');

    // "A:X=" is meant to delete thermal alert at index X
    if (cfg[4] == 0) {
#ifdef DEBUG_MQTT
        twr_log_debug("config_set_thermal_alert(): deleting thermal alert at index %d", index);
#endif
        eeprom.thermal_alert[index].thermal_zone = 0;
        eeprom.thermal_alert[index].gt = false;
        eeprom.thermal_alert[index].temperature = 0;
        eeprom.thermal_alert[index].text[0] = 0;
        return true;
    }

    // parse thermal zone
    ta_validate_char(4, 'Z');
    hex_at(zone_index, 5);
    ta_validate_char(6, '>');
    int end = float_end(&cfg[7]);
    cfg[7+end] = 0;
    float temperature = strtof(&cfg[7], NULL);
    char *text = NULL;
    if (cfg[7+end] == '#') {
        text = &cfg[8+end];
        if (strlen(text) > THERMAL_ALERT_TEXT_LENGTH-1) {
            // name too long - log error and cut it
            twr_log_error("config_set_thermal_alert(): thermal alert text too long, cutting to %d chars (%s)", THERMAL_ALERT_TEXT_LENGTH-1, text);
            text[THERMAL_ALERT_TEXT_LENGTH-1] = 0;
        }
    }

    // store thermal alert to eeprom at given index
    eeprom.thermal_alert[index].thermal_zone = zone_index;
    eeprom.thermal_alert[index].gt = true;
    eeprom.thermal_alert[index].temperature = temperature;
    if (text != NULL)
        strncpy(eeprom.thermal_alert[index].text, text, THERMAL_ALERT_TEXT_LENGTH);
    else
        eeprom.thermal_alert[index].text[0] = 0;

#ifdef DEBUG_MQTT
    twr_log_debug("config_set_thermal_alert(): parsed thermal alert: index=%d, zone=%d, gt=%d, temp=%.1f, text=%s", index, zone_index, true, temperature, text);
#endif
    
        return true;
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
        case 'F':
            ok = config_set_sensor_fixed(cfg, topic);
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
        case 'A':
            ok = config_set_thermal_alert(cfg, topic);
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

bool pub_config_onewire(uint8_t index) {
    uint8_t i;
    for (i = 0; i < OW_MAX_SLAVES; i++) {
        if (eeprom.sensor_onewire_map[i].address != 0) {
            if (index == 0)
                break;
            index--;
        }
    }
    if (i < OW_MAX_SLAVES && eeprom.sensor_onewire_map[i].address != 0) {
        sprintf(mqtt_bufer, "O:%d=%016llX", i, eeprom.sensor_onewire_map[i].address);
        if (!twr_radio_pub_string("cfg/ow", mqtt_bufer)) {
            twr_log_error("pub_config_onewire(): publish failed");
        }
        return true;
    }
    return false;
}

bool pub_config_fixed(uint8_t index) {
    uint8_t i;
    for (i = 0; i < SENSOR_CONST_COUNT; i++) {
        if (eeprom.sensor_const[i].temperature != 0) {
            if (index == 0)
                break;
            index--;
        }
    }
    if (i < SENSOR_CONST_COUNT) {
        sprintf(mqtt_bufer, "F:%d=%.1f", i, eeprom.sensor_const[i].temperature);
        if (!twr_radio_pub_string("cfg/fx", mqtt_bufer)) {
            twr_log_error("pub_config_fixed(): publish failed");
        }
        return true;
    }
    return false;
}

bool pub_config_thermal_zone(uint8_t index) {
    uint8_t i;
    for (i = 0; i < MAX_THERMAL_ZONES; i++) {
        if (eeprom.thermal_zone[i].sensors[0] != 0 || eeprom.thermal_zone[i].sensors[1] != 0) {
            if (index == 0)
                break;
            index--;
        }
    }
    if (i < MAX_THERMAL_ZONES && (eeprom.thermal_zone[i].sensors[0] != 0 || eeprom.thermal_zone[i].sensors[1] != 0)) {
        sprintf(mqtt_bufer, "Z:%d=%02X%c%02X#%s", i, eeprom.thermal_zone[i].sensors[0], eeprom.thermal_zone[i].absolute ? '|' : '-', eeprom.thermal_zone[i].sensors[1], eeprom.thermal_zone[i].name);
        if (!twr_radio_pub_string("cfg/tz", mqtt_bufer)) {
            twr_log_error("pub_config_thermal_zone(): publish failed");
        }
        return true;
    }
    return false;
}

bool pub_config_fan_group(uint8_t index) {
    uint8_t i;
    for (i = 0; i < MAX_FAN_GROUPS; i++) {
        if (eeprom.fan_group[i].fans != 0) {
            if (index == 0)
                break;
            index--;
        }
    }
    if (i < MAX_FAN_GROUPS && eeprom.fan_group[i].fans != 0) {
        sprintf(mqtt_bufer, "G:%d=%d%d%d%d%d#%s", i, (eeprom.fan_group[i].fans>>4)&1, (eeprom.fan_group[i].fans>>3)&1, (eeprom.fan_group[i].fans>>2)&1, (eeprom.fan_group[i].fans>>1)&1, (eeprom.fan_group[i].fans>>0)&1, eeprom.fan_group[i].name);
        if (!twr_radio_pub_string("cfg/fg", mqtt_bufer)) {
            twr_log_error("pub_config_fan_group(): publish failed");
        }
        return true;
    }
    return false;
}

bool pub_config_map_rule(uint8_t index) {
    uint8_t i;
    for (i = 0; i < MAX_MAP_RULES; i++) {
        if (eeprom.map_rule[i].thermal_zone != 0 || eeprom.map_rule[i].fan_group != 0) {
            if (index == 0)
                break;
            index--;
        }
    }
    if (i < MAX_MAP_RULES && (eeprom.map_rule[i].thermal_zone != 0 || eeprom.map_rule[i].fan_group != 0)) {
        sprintf(mqtt_bufer, "R:%d=Z%d(%.1f,%.1f)G%d(%.1f,%.1f)#%s", i, eeprom.map_rule[i].thermal_zone, eeprom.map_rule[i].temperature[0], eeprom.map_rule[i].temperature[1], eeprom.map_rule[i].fan_group, eeprom.map_rule[i].speed[0], eeprom.map_rule[i].speed[1], eeprom.map_rule[i].name);
        // mqtt_bufer[10] = 0;
        if (!twr_radio_pub_string("cfg/mr", mqtt_bufer)) {
            twr_log_error("pub_config_map_rule(): publish failed");
        }
        return true;
    }
    return false;
}

bool pub_config_thermal_alert(uint8_t index) {
    uint8_t i;
    for (i = 0; i < MAX_THERMAL_ALERTS; i++) {
        if (eeprom.thermal_alert[i].thermal_zone != 0) {
            if (index == 0)
                break;
            index--;
        }
    }
    if (i < MAX_THERMAL_ALERTS && eeprom.thermal_alert[i].thermal_zone != 0) {
        sprintf(mqtt_bufer, "A:%d=Z%d%c%.1f#%s", i, eeprom.thermal_alert[i].thermal_zone, eeprom.thermal_alert[i].gt ? '>' : '<', eeprom.thermal_alert[i].temperature, eeprom.thermal_alert[i].text);
        mqtt_bufer[50] = 0;
        if (!twr_radio_pub_string("cfg/ta", mqtt_bufer)) {
            twr_log_error("pub_config_thermal_alert(): publish failed");
        }
        return true;
    }
    return false;
}

#define PUB_TYPE_OW 0
#define PUB_TYPE_SF 1
#define PUB_TYPE_TZ 2
#define PUB_TYPE_FG 3
#define PUB_TYPE_MR 4
#define PUB_TYPE_TA 5
uint8_t pub_index = 0;
uint8_t pub_type = PUB_TYPE_OW;
bool pub_config_in_progress = false;
twr_scheduler_task_id_t pub_callback = 0;

bool pub_config_next(void) {
#ifdef DEBUG_MQTT
    twr_log_debug("pub_config_next(): pub_type: %d, pub_index: %d", pub_type, pub_index);
#endif
    if (!pub_config_in_progress)
        return false;
    switch (pub_type) {
        case PUB_TYPE_OW:
            if (pub_config_onewire(pub_index++))
                return true;
            pub_type = PUB_TYPE_SF;
            pub_index = 0;
            __attribute__ ((fallthrough));
        case PUB_TYPE_SF:
            if (pub_config_fixed(pub_index++))
                return true;
            pub_type = PUB_TYPE_TZ;
            pub_index = 0;
            __attribute__ ((fallthrough));
        case PUB_TYPE_TZ:
            if (pub_config_thermal_zone(pub_index++))
                return true;
            pub_type = PUB_TYPE_FG;
            pub_index = 0;
            __attribute__ ((fallthrough));
        case PUB_TYPE_FG:
            if (pub_config_fan_group(pub_index++))
                return true;
            pub_type = PUB_TYPE_MR;
            pub_index = 0;
            __attribute__ ((fallthrough));
        case PUB_TYPE_MR:
            if (pub_config_map_rule(pub_index++))
                return true;
            pub_type = PUB_TYPE_TA;
            pub_index = 0;
            __attribute__ ((fallthrough));
        case PUB_TYPE_TA:
            if (pub_config_thermal_alert(pub_index++))
                return true;
            __attribute__ ((fallthrough));
        default:
            break;
    }
    pub_type = PUB_TYPE_OW;
    pub_index = 0;
    pub_config_in_progress = false;
    twr_scheduler_unregister(pub_callback);
    return false;
}

void pub_config_callback(void * param) {
    if (pub_config_next())
        twr_scheduler_plan_current_relative(MQTT_CONFIG_DUMP_INTERVAL);
}


bool pub_config_start(void) {
    pub_config_in_progress = true;
    pub_type = PUB_TYPE_OW;
    pub_index = 0;
#ifdef DEBUG_MQTT
    twr_log_debug("pub_config_start(): starting config publish");
#endif
    pub_callback = twr_scheduler_register(pub_config_callback, NULL, twr_tick_get() + MQTT_CONFIG_DUMP_INTERVAL);
    return pub_config_next();
}

// Dumps configuration as individual MQTT messages in same format as expected by config_set().
void config_get(uint64_t *id, const char *topic, void *value, void *param) {
#ifdef DEBUG_MQTT
    twr_log_debug("config_get(): MQTT topic: '%s'", topic);
#endif
    pub_config_start();
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


/* static void radio_event_handler(twr_radio_event_t event, void *event_param)
{
    (void)event_param;

    // TWR_RADIO_EVENT_TX_DONE

//     switch (event) {
//         case TWR_RADIO_EVENT_TX_DONE:
// #ifdef DEBUG_MQTT
//             twr_log_debug("Radio: TX done");
// #endif
//             // check if config dump is in progress and continue
//             if (pub_config_in_progress) {
//                 pub_config_next();
//             }
//             break;
//     }

    // if (event == TWR_RADIO_EVENT_ATTACH)
    // {
    //     bc_log_debug("Radio: attach");
    // }
    // else if (event == TWR_RADIO_EVENT_DETACH)
    // {
    //     bc_log_debug("Radio: detach");
    // }
    // else if (event == TWR_RADIO_EVENT_INIT_DONE)
    // {
    //     // my_id = bc_radio_get_my_id();
    //     // bc_log_debug("Radio: init done, id = 0x%" PRIx64, my_id);
    // }
}
 */

void mqtt_init(void) {
    // Initialize radio - keep listening as we are not on battery
    twr_radio_init(TWR_RADIO_MODE_NODE_LISTENING);
    twr_radio_set_subs(mqtt_subs, sizeof(mqtt_subs)/sizeof(mqtt_subs[0]));
    // twr_radio_set_event_handler(radio_event_handler, NULL);
   // Send radio pairing request
    twr_radio_pairing_request("water-cooler", FW_VERSION);
}