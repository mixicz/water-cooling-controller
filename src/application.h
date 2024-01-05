#ifndef _APPLICATION_H
#define _APPLICATION_H

#include <twr.h>
#include <bcl.h>

#include <fan.h>

// Application version
// #define FW_VERSION "1.0"
#define EEPROM_SIGNATURE        0x57434330
#define EEPROM_VERSION          0x01

// #define DEBUG   true   // not needed, DEBUG is defined in ninja build file

#ifdef DEBUG
// #define DEBUG_FAN_CALIBRATION   true
// #define DEBUG_ONEWIRE           true
// #define DEBUG_ADC               true
#define DEBUG_ADC_CALIBRATION   true
// #define DEBUG_PCA9685           true
#define DEBUG_CONTROL           true
#endif

// control logic
#define CONTROL_LOOP_PERIOD 1000

// sensor ID is 8 bit value with upper 4 bits reserved for sensor bus type and lower 4 bits for sensor address
#define SENSOR_BUS_I2C 0x00
#define SENSOR_BUS_ONEWIRE 0x10
#define SENSOR_BUS_ADC 0x20
#define SENSOR_BUS_CONST 0xF0
#define SENSOR_MAX_COUNT 16

// 1-wire configuration
# define OW_MAX_SLAVES 8

// onewire sensor mapping to provide constant ID for each sensor
typedef struct {
    uint8_t index;
    uint64_t address;
} sensor_onewire_t;

// fixed temperature pseudo-sensor
#define SENSOR_CONST_COUNT 16

typedef struct {
    float temperature;
} sensor_const_t;

// thermal zone configuration
#define MAX_THERMAL_ZONES 16
#define THERMAL_ZONE_NAME_LENGTH 16

typedef struct {
    uint8_t sensors[2];
    bool absolute;
    char name[THERMAL_ZONE_NAME_LENGTH];
} thermal_zone_t;

// fan group configuration
#define MAX_FAN_GROUPS 16
#define FAN_GROUP_NAME_LENGTH 16

typedef struct {
    uint8_t fans;   // bit mask of fans in this group
    char name[FAN_GROUP_NAME_LENGTH];
} fan_group_t;

// map rules - map thermal zones to fan groups
#define MAX_MAP_RULES 16
#define MAP_RULE_NAME_LENGTH 16

// maps temperature range to fan speed range with linear interpolation
typedef struct {
    uint8_t thermal_zone;
    uint8_t fan_group;
    float temperature[2];
    float speed[2];
    char name[MAP_RULE_NAME_LENGTH];
} map_rule_t;

// configuration structure
typedef struct {
    uint8_t version;
    uint16_t fan_ramp_up_step_time; // time in ms between 2 steps of FAN speed ramp up
    uint16_t fan_ramp_down_step_time; // time in ms between 2 steps of FAN speed ramp down
    uint16_t pump_ramp_up_step_time; // time in ms between 2 steps of pump speed ramp up
    uint16_t pump_ramp_down_step_time; // time in ms between 2 steps of pump speed ramp down
} config_t;

// ADC configuration
#define ADC_CHANNEL_COUNT 6
// TODO - replace with real measured values
#define ADC_DEFAULT_OFFSET 0.0
#define ADC_DEFAULT_GAIN 1.0

// ADC calibration data
typedef struct {
    bool present;
    bool calibrated;
    float offset;
    float gain;
} adc_calibration_t;

// temperature sensors runtime data
typedef struct {
    float temp_i2c[1];
    float temp_onewire[OW_MAX_SLAVES];
    float temp_adc[ADC_CHANNEL_COUNT];
} sensor_runtime_t;

typedef struct {
    uint32_t signature;
    config_t config;
    // fan_config_t fan_config[MAX_FANS];
    fan_user_config_t fan_user_config[MAX_FANS];
    fan_calibration_t fan_calibration[MAX_FANS];
    adc_calibration_t adc_calibration[ADC_CHANNEL_COUNT];
    sensor_onewire_t sensor_onewire_map[OW_MAX_SLAVES];
    sensor_const_t sensor_const[SENSOR_CONST_COUNT];
    thermal_zone_t thermal_zone[MAX_THERMAL_ZONES];
    fan_group_t fan_group[MAX_FAN_GROUPS];
    map_rule_t map_rule[MAX_MAP_RULES];
} eeprom_t;

extern eeprom_t eeprom;
extern config_t *config;
extern fan_config_t *fan_config;
extern fan_user_config_t *fan_user_config;
extern fan_calibration_t *fan_calibration;
extern adc_calibration_t *adc_calibration;


#endif
