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
#define DEBUG_ONEWIRE           true
// #define DEBUG_ADC               true
#define DEBUG_ADC_CALIBRATION   true
// #define DEBUG_PCA9685           true
#endif

// configuration structure
typedef struct {
    uint8_t version;
    uint8_t fans;
    uint16_t fan_ramp_up_step_time; // time in ms between 2 steps of FAN speed ramp up
    uint16_t fan_ramp_down_step_time; // time in ms between 2 steps of FAN speed ramp down
    uint16_t pump_ramp_up_step_time; // time in ms between 2 steps of pump speed ramp up
    uint16_t pump_ramp_down_step_time; // time in ms between 2 steps of pump speed ramp down
} config_t;

// ADC configuration
#define ADC_CHANNEL_COUNT 6
// #define ADC_CHANNEL_COUNT 4

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

typedef struct {
    uint32_t signature;
    config_t config;
    fan_config_t fan_config[MAX_FANS];
    fan_user_config_t fan_user_config[MAX_FANS];
    fan_calibration_t fan_calibration[MAX_FANS];
    adc_config_t adc_config[ADC_CHANNEL_COUNT];
    adc_calibration_t adc_calibration[ADC_CHANNEL_COUNT];
} eeprom_t;

extern eeprom_t eeprom;
extern config_t *config;
extern fan_config_t *fan_config;
extern fan_user_config_t *fan_user_config;
extern fan_calibration_t *fan_calibration;
extern adc_config_t *adc_config;
extern adc_calibration_t *adc_calibration;


#endif
