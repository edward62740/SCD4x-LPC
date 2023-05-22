# SCD4x Low-Power Calibration (LPC) Algorithm [in progress]
The SCD4x sensor has a built-in Automatic Self-Calibration (ASC) that is not available for ultra-low power applications that require power-down of the sensor between measurements.

This C++ algorithm attempts to emulate the behaviors of the ASC in single-shot mode, based on the ASC Application Note (p. 2-5,11-13). The key assumption is that the lowest measured co2 is no less than SCD4X_LPC_BASELINE_CO2_PPM.<br> 
Do **not** use this (or the ASC) if the assumption does not hold for the application requirements.<br>

![Graph](https://github.com/edward62740/SCD4x-LPC/blob/master/graph.png)
<br>*ASC Application Note (p. 5)*
<br>

It also is designed to run asynchonously (from the sensor) and delays in between function calls are up to the user to enforce. This algorithm provides some safety by taking in a platform defined millisecond timer in the constructor and blocking function calls for the given sensor execution durations (specified in SCD4x datasheet p.8). The HAL function sensirion_i2c_hal_sleep_usec must be implemented as such:
```
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
	if(useconds > 1000) return;
    platform_delay_us(useconds); // platform specific delay or RTC sleep
}
```
The only function calls that are not handled by this method are the FRC (400ms) and self-test(10000ms, never called by this lib). These need to be manually modified by the user if maximum power savings are needed.

In general, this algorithm has a FSM that goes through 3 states:
- First: First reading calibration
- Initial: Initial period of 576 measurements
- Standard: Standard period of 2016 measurements (recurring)

<br> Wherein the FRC is allowed to run once per period, when the reading is below baseline OR at the end of the period, whichever is earlier.
The calibration is defined by a signed offset value for each period, where a measurement point $t \in [0, \text{NUM MEAS}] \cap \mathbb{Z}$, with $f(t)$ representing the co2 reading at point $t$:

```math
\text{{offset}} = \begin{cases}
    \text{{BASELINE}} - f(t), & \text{{if }} f(t) < \text{{BASELINE}} \
    \text{{BASELINE}} - \min(f(t)), & \text{{otherwise}}
\end{cases}

```
Note that in the case where $f(t) < \text{{BASELINE}}$, accuracy may only be restored over a few periods, as the calibration point may represent only a local minimum.

All the thresholds/configs are user-definable in the header file.

Initialization requires the necessary SCD4x driver function pointers to be passed to the constructor. This allows for the generic C driver to be used, or C++ class instance methods.
```
LPC::SCD4X sensor(scd4x_wake_up, scd4x_power_down, scd4x_measure_single_shot,
		 scd4x_get_data_ready_flag, scd4x_read_measurement, scd4x_perform_forced_recalibration,
		 scd4x_persist_settings, sl_sleeptimer_get_tick_count);
```

It is recommended to disable ASC and set persistent settings prior to using this algorithm.
