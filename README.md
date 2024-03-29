# Firmware for Water Cooling Controller using HARDWARIO TOWER Core Module

TODO

## TODO list
- :heavy_check_mark: 1-wire thermometer configuration
- :heavy_check_mark: configurable names for all entities
- :heavy_check_mark: ADC calibration
- :heavy_check_mark: ADC periodic temperature readings
- :question: status LEDs via Adafruit PCA9685 PWM driver - might get replaced by DRGB support instead to drive RGB lightning and use it for alerts and statuses as well (changing color based on sensors, blinking on alerts / calibration)
- :heavy_check_mark: save config and calibration data to EEPROM
- :heavy_check_mark: load config and calibration data from EEPROM
- :heavy_check_mark: automatic FAN calibration
- ~~performance profiles for FANs~~ (not really necessary)
- :heavy_check_mark: MQTT - reporing temperatures
- MQTT - reporting alarms and statuses
    - :heavy_check_mark: unusually high temperature, 
    - FAN failure, 
    - :heavy_check_mark: start/stop calibration (ADC only),
- MQTT - commands:
    - start/stop calibration,
    - fill mode
- MQTT - set/get config values:
    - :heavy_check_mark: control logic,
    - ADC calibration values,
    - PWM ramp times
- :heavy_check_mark: LED control:
    - on - NTC calibration in progress, waiting for temperature stabilization
    - brief flashes - NTC calibration in progress, waiting for temperature change
    - blinking - fan calibration in progress
    - off - normal operation

## features
* cooling control based on measured temperatures inside and outside of a cooling loop instead of OS reported values,
* fan detection and calibration,
* NTC thermistor detection and calibration (requires at least one already calibrated sensor on 1-wire)
* 1-wire temperature sensor detection
* HW features:
    * 6 NTC thermistor ports,
    * 5 PWM ports for fans and pump,
    * built-in temperature sensor on core module,
    * 1-wire bus for additional temperature sensors,
    * I2C bus for additional PWM LEDs via PCA9685,

## HW description
### Default sensor assignment
Any connected sensors not mentioned here are not used to control the cooling by default, but they are reported over MQTT and you can setup yourown rules using them.

#### I2C
- `00` – only onboard sensor, not used for cooling control, but reported over MQTT

#### 1-wire
- `10` – outside ambient temperature (preferably placed in front of radiator intake to have air flowing around it),
- `11` – inside case ambient temperature (preferably in upper part of case in front of venting holes or top radiator),

#### ADC / NTC
> [!NOTE]
> We assume loop order (with sensors in parentheses): pump → (`20`)GPU(`21`) → small radiator → (`22`)CPU(`23`) → large radiator → pump, but in reality it is mostly about naming zones and most important thing is, that `20` should be placed at coolest part of loop and `23` at most warm part. It does not matter at all if there is an radiator between GPU and CPU or the order of CPU and GPU. Just assume we named first block in loop as GPU and second as CPU.
- `20` - before GPU,
- `21` - after GPU,
- `22` - before CPU,
- `23` - after CPU,

## Concepts
- **sensor** – value from temperature sensor, each sensor have unique ID in form of 8-bit hexadecimal value, where bus is higher 4-bits and number are lower 4-bits:
    - I2C sensor (bus `0`, ID of internal onboard sensor is `00`, no other I2C sensors are supported at this time)
    - 1-wire sensor (bus `1`, number is configurable, by default it is given by order of detection, which is ascending by serial number)
    - NTC sensor (bus `2`, number is given by ADC channel 0..5)
    - fixed value (bus `F`), there are predefined numbers:
        - `0`: 0°C
        - `1`-`3`: undefined - reserved for future use (0°C at this moment, but it might change in future)
        - `4`-`F`: user configurable, default value is 0°C
- **thermal zone** – delta of 2 sensors (absolute value) in °C
- **fan group** – set of 0..N PWM outputs
    - each fan can be part of multiple groups
- **map rule** – maps temperature range from thermal zone to speed range in fan group
    - all rules are always processed and maximum resulting RPM is always used,
    - if thermal zone value is out of range, rule is skipped,
    - linear interpolation is used for mapping,
- **thermal alert** - configurable tresholds for thermal zones which will trigger alert message on USB and MQTT (TODO)

Notes:
- target RPM is defined as percentage (value range 0..100) and is converted to real RPM for individual fans independently. So if there are 2 fans in group with max RPM of 1000 and 2000 respectively and RPM is set to 50%, fans will be set to 500 and 1000 RPM,
- target RPM of each individual fan is set to maximum RPM value of all processed rules,
- if no target RPM is determined for the fan, default RPM value of the fan is used,
- sensors, zones, groups and rules can optionally be named, this is used in debug information over USB and messages over MQTT

## MQTT communication
> [!NOTE] 
> We deliberately use short topic names in order to allow for larger payload, as topic+message must fit into 60 byte radio message.

### `wc/-/cfg/*`
- `wc/-/cfg/set` - sets configuration value to any of below options (confirmation message on `cfg/set` topic),
- `wc/-/cfg/get` - dumps all configuration to MQTT `cfg/#` topics (e.g. `cfg/ow`, `cfg/sf`, `cfg/tz`),

### Sensor onewire
- maps 1-wire sensor address to sensor ID
- string representation as `O:<index>=<address>`, where
    - *index* - single hex digit (`0`..`F`) as 1-wire sensor index,
    - *address* - 64-bit hexadecimal number as 1-wire device address

### Sensor fixed
- sets fixed sensor value
- string representation as `F:<index>=<temperature>`, where
    - *index* - single hex digit (`0`..`F`) as fixed sensor index,
    - *temperature* - float value in °C

### Thermal zone definition
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
    - `Z:2=25-F0#EOL water temp` - temperature of water at the end of the cooling loop (difference between last NTC sensor in loop and constant 0°C).

### Fan group definition
- string representation as `G:<index>=<binary value>#<name>`, where
    - each bit in 5-bit binary value represent PWM ports, with `1` meaning the PWM output is part of the group and leftmost bit has highest index,
- example:
    - `G:0=10000#pump` - only PWM port 5 (pump) is part of the group,
    - `G:1=01111#all radiators` - PWM ports 1 to 4 are part of the group
    - `G:2=00011#front radiator` - PWM ports 1 and 2 are part of the group

### Map rule definition
- string representation as `R:<index>=Z<zone>(t1,t2)G<group>(s1,s2)#<name>`, where:
    - *index* - single hex digit (`0`..`F`) as map rule index,
    - *zone* - single hex digit (`0`..`F`) as thermal zone index,
    - *group* - single hex digit (`0`..`F`) as fan group index,
    - *t1*/*t2* - start/end of temperature range as float in °C,
        - for temperatures below or above specified interval, *s1* or *s2* speed will be used respectively,
    - *s1*/*s2* - start/end of relative speed range as float in <0..1> interval,
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

## Controls
Upon first start, all connected fans will be detected and calibrated.

### button
- short press: run detection of 1-wire devices
- long press: start NTC/ADC calibration process
- long press followed by short press before finishing NTC calibration: start [fan calibration process](#fan-calibration) (and cancel NTC calibration started by long press)
- long press followed by long press: cancel NTC calibration started by long press

### LED
- on - NTC calibration in progress, waiting for temperature stabilization
- brief flashes - NTC calibration in progress, waiting for temperature change
- fast blinking - fan calibration in progress
- slow blinking - fill mode
- off - normal operation

## Calibration
### Fan calibration
> [!CAUTION]
> If possible, do not run fan calibration process while PC is turned on. Fans and pumps are going to be spinned down to 0% PWM rate and as some models (all Noctua fans for example) are not capped to 20%, they will completely stop spinning for a while. This might potentially cause damage to HW as even throttled down CPU/GPU produces heat.

When started, all PWM outputs are set to 100% for a while and then slowly spinned down in 10% steps. At each step, controller waits for RPM to stabilize before measuring calibration values. After measurement is done, calibration values are computed and PWM outputs are set to default speed.

Calibration evaluates following scenarios:
- PWM to RPM curve, using computed PWM value for desired RPM,
- disconnected PWM outputs (zero RPM on particular poutput through whole process),
- fans without PWM control (constant non-zero RPM),
- lower PWM range bound – specification defines only 20%-100% range, with 0%-20% as undefined behavior. Fans typically caps RPM at 20% for PWM values <20%, while some of them are able to go below 20%. When fan is able to go lower, controller will use all available range up to 10% / 0% PWM rate depending on detected RPM. When PWM output is marked as water pump (default is #5), 20% is always used as minimum PWM value regardless of detected settings.

### NTC calibration
Calibration is done by placing at least one 1-wire temperature sensor together with all connected NTC sensors to medium with constant temperature, waiting for readings to stabilize and repeat process with different temperature. Temperature difference must be at least 20°C. Best is to use cold and hot water and fix sensors together with decent spacing to allow more reliable temperature stabilization.

Calibration process step by step:
1. place sensors to cold water,
1. long-press button on core module,
1. wait for message about finished first measurement on USB / MQTT or LED start flashing,
1. place sensors to hot water,
1. wait for message about finished calibration on USB / MQTT or LED going completely off (it will stop flashing and turn on first, when large enough temperature difference is detected).

## Fill mode
> [!CAUTION]
> Never use filling mode with computer turned on! all fans and pump are disabled by default, only spinning up pump on button press.

In order to ease filling up water loop, it is possible to use special fill mode. In this mode, fan are at minimum possible RPM (0% PWM – stopped if they support it) and pump is controlled by button on core module, switching PWM rate between 0% and configured value (default 30%) on each button press. LED blinks slowly in fill mode.

Fill mode is enabled and controlled over MQTT topic `wc/-/cmd/fill` and `wc/-/cmd/fill-speed` with string commands:
- `wc/-/cmd/fill` = `"start"` - starts fill mode with still pump and 30% PWM,
- `wc/-/cmd/fill` = `"stop"` - ends filling mode and resumes normal operation,
- `wc/-/cmd/fill-speed` = `<float>` - sets speed of active pump in interval 0..1 (e.g.)

## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT/) - see the [LICENSE](LICENSE) file for details.

---

Based on Hardwario Tower by [**HARDWARIO a.s.**](https://www.hardwario.com/).
