# SCD4x Low-Power Calibration (LPC) Algorithm [in progress]
The SCD4x sensor has a built-in Automatic Self-Calibration (ASC) that is not available for ultra-low power applications that require power-down of the sensor between measurements.

This C++ library attempts to emulate the behaviors of the ASC in single-shot mode, based on the ASC Application Note (p. 2-5,11-13). The key assumption is that the lowest measured co2 is no less than SCD4X_LPC_BASELINE_CO2_PPM.<br>
It also is designed to run asynchonously and delays in between function calls are up to the user to enforce. The HAL function sensirion_i2c_hal_sleep_usec must do nothing.

In general, this library has a FSM that goes through 3 states:
- First: First reading calibration
- Initial: Initial period of 576 measurements
- Standard: Standard period of 2016 measurements (recurring)

<br> Wherein the FRC is allowed to run once per period, when the reading is below baseline OR at the end of the period, whichever is earlier.
The maximum offset per period is also varied.

All the thresholds/configs are user-definable in the header file.

Initialization requires the necessary SCD4x driver function pointers to be passed to the constructor.


