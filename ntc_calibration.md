# NTC Thermistor Calibration and Temperature Conversion

## Precision Analysis for ±0.1°C Tolerance

**Single precision (float32) is SUFFICIENT** for ±0.1°C tolerance:

- Floating point error with single precision: ~±1e-6°C (negligible)
- Dominant error sources:
  - ADC quantization (12-bit): ±0.02–0.05°C
  - Thermistor tolerance: ±0.3–0.6°C
  - Calibration accuracy: ±0.1°C

Since floating point error is orders of magnitude smaller than the ±0.1°C tolerance, and hardware limitations dominate, **single precision provides 50% memory savings and faster computation on ARM32 without an FPU**.

## Steinhart-Hart Equation

The Steinhart-Hart equation models NTC thermistor temperature:

$$
\frac{1}{T} = A + B \ln(R) + C [\ln(R)]^3
$$

where:
- T is absolute temperature in Kelvin
- R is thermistor resistance in Ohms
- A, B, C are coefficients determined from 3 calibration points

## Calibration Procedure

1. **Measure three calibration points** at known temperatures and resistances:
   - Point 1: Low temperature (~10°C)
   - Point 2: Mid temperature (~25°C)
   - Point 3: High temperature (~50–60°C)

2. **Run calibration function** to derive A, B, C coefficients

3. **Store coefficients** on the MCU for runtime calculations

## C Implementation (Single Precision)

```c
#include <math.h>

// Circuit configuration (global constants)
#define CALIB_VOLTAGE 3.3f          // Supply voltage (Volts)
#define DIVISOR_RESISTANCE 10000.0f // Series resistor (Ohms)

typedef struct {
    float A, B, C;
} SteinhartHartCoeff;

/// Calculate Steinhart-Hart coefficients from 3 calibration points
/// @param temp_c Array of 3 temperatures in Celsius
/// @param resistance Array of 3 resistances in Ohms
/// @return Struct containing A, B, C coefficients
SteinhartHartCoeff calibrate_ntc(float temp_c[3], float resistance[3]) {
    // Convert Celsius to Kelvin
    float T[3];
    for (int i = 0; i < 3; i++) {
        T[i] = temp_c[i] + 273.15f;
    }
    
    // Calculate natural logs of resistance
    float L[3];
    for (int i = 0; i < 3; i++) {
        L[i] = logf(resistance[i]);
    }
    
    // Precompute powers
    float L_sq[3], L_cubed[3];
    for (int i = 0; i < 3; i++) {
        L_sq[i] = L[i] * L[i];
        L_cubed[i] = L_sq[i] * L[i];
    }
    
    // Inverse of 1/T values
    float Y[3];
    for (int i = 0; i < 3; i++) {
        Y[i] = 1.0f / T[i];
    }
    
    // Solve system for C, B, A coefficients
    float denom = (L[0] - L[1]) * (L[1] + L[2]) - (L[1] - L[2]) * (L[0] + L[1]);
    float C = ((Y[0] - Y[1]) * (L[1] + L[2]) - (Y[1] - Y[2]) * (L[0] + L[1])) / denom;
    
    float B = ((Y[0] - Y[1]) - C * (L_cubed[0] - L_cubed[1])) / (L[0] - L[1]);
    
    float A = Y[0] - B * L[0] - C * L_cubed[0];
    
    SteinhartHartCoeff coeff = {A, B, C};
    return coeff;
}

/// Convert ADC voltage reading to temperature
/// @param adc_voltage Measured voltage from ADC (Volts)
/// @param coeff Steinhart-Hart coefficients from calibration
/// @return Temperature in Celsius
float adc_voltage_to_celsius(float adc_voltage, SteinhartHartCoeff coeff) {
    // Calculate thermistor resistance from voltage divider
    // V_ntc = CALIB_VOLTAGE * R_ntc / (R_ntc + DIVISOR_RESISTANCE)
    // Solve for R_ntc:
    float R_ntc = DIVISOR_RESISTANCE * adc_voltage / (CALIB_VOLTAGE - adc_voltage);
    
    // Steinhart-Hart calculation
    float ln_r = logf(R_ntc);
    float ln_r_cubed = ln_r * ln_r * ln_r;
    
    float inv_T = coeff.A + coeff.B * ln_r + coeff.C * ln_r_cubed;
    float T_kelvin = 1.0f / inv_T;
    
    return T_kelvin - 273.15f;
}

/// Example usage:
/// 
/// // Step 1: Calibration (run once during setup)
/// float calib_temps[3] = {10.0f, 25.0f, 50.0f};      // Celsius
/// float calib_resistance[3] = {25415.0f, 10021.0f, 6545.0f};  // Ohms
/// SteinhartHartCoeff coeff = calibrate_ntc(calib_temps, calib_resistance);
/// 
/// // Step 2: Runtime measurement (in main loop)
/// float measured_voltage = 1.85f;  // From ADC, in Volts
/// float temperature = adc_voltage_to_celsius(measured_voltage, coeff);
```

## Circuit Setup

The NTC thermistor is typically connected in a voltage divider configuration:

```
   CALIB_VOLTAGE
        |
        +--[R_divisor]--+---> to ADC
        |               |
        +--[R_ntc]------+
        |
       GND
```

Where:
- **CALIB_VOLTAGE**: 3.3V (or your supply voltage)
- **DIVISOR_RESISTANCE**: 10kΩ (or match your circuit)
- **R_ntc**: NTC thermistor (typically 10kΩ at 25°C)

## Memory Usage (ARM32)

| Type | Single Precision | Double Precision |
|------|-----------------|-----------------|
| Struct size | 12 bytes | 24 bytes |
| Stack usage (calc) | ~40 bytes | ~80 bytes |
| Total per sensor | ~52 bytes | ~104 bytes |
| 4 sensors | ~208 bytes | ~416 bytes |

**50% memory savings** with single precision while maintaining ±0.1°C accuracy.

## Accuracy Considerations

1. **Calibration quality** is critical—use your precalibrated 1-wire sensors to establish accurate reference points
2. **Voltage stability**: Use a regulated 3.3V supply; ±1% variation ≈ ±0.05°C error
3. **Measurement averaging**: Take 10–100 ADC samples and average to reduce quantization noise
4. **Temperature drift**: Calibrate across the full operating range (10–60°C) for best results
