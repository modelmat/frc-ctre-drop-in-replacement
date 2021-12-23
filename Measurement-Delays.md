# Measurement Delays

Both CTRE Phoenix and RevRobotics[^1] motor controllers by default have moving average filters on their velocity measurements.
These induce a measurement delay which can make an LQR controller unstable.

[^1]: Not discussed in detail here, though the math is the same. For more information on their behaviour, see the sysid PR listed under sources.

## Calculating Delay

### Measurement Period Delay

The default measurement period (the time between position measurements used to calculate velocity) for a CTRE Phoenix motor controller is `100 ms`.
This can be changed with the [`motor.ConfigVelocityMeasurementPeriod()`](https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#af5ced9b7f66b1a48b8c9d2816fa75f07) function.

The delay caused by this is `Period / 2`.[^2]
For the default configuration, this is `100 / 2` = `50 ms` of delay.

### Moving Average Delay

The default measurement window size (i.e. number of samples or "taps") for a CTRE Phoenix Motor Controller is `64` (samples).
This can be changed with `encoder.SetSamplesToAverage(int)`, which calls the [`motor.ConfigVelocityMeasurementWindow(int)`](https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#ad5211c87f0b92a6dc623e1f33e9b6cb9) function; values are constrained to powers of two between 1 and 64.

The average delay (in units of samples) for *n* samples is given by `(n-1)/2`[^2].
For CTRE Phoenix motor controllers, a sample is taken every `1 ms`, so the delay is `(n-1)/2 ms`.
For the default configuration, this is a delay of `(64-1)/2 = 31.5 ms`.

[^2]: For the proof, see the lines mentioned "specifically" in the sysid PR.

### Overall Delay

The overall delay is the sum of the sample period and moving average delays.
For the default CTRE Phoenix configuration, this works out to a measurement delay of `50 + 31.5 = 81.5 ms`.

Rough code to determine the overall delay would be:

```cpp
double measurementDelay(int windowSize, double measurementPeriod) {
    return (static_cast<double>(windowSize) - 1.0)/2 + measurementPeriod/2;
}
```

## Adjusting for Delay

The delay measurement can be adjusted for with [`LinearQuadraticRegulator.LatencyCompensate()`]((https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/controller/LinearQuadraticRegulator.html#latencyCompensate(edu.wpi.first.wpilibj.system.LinearSystem,double,double))).


## Sources / Further Reference:

- [CTRE Phoenix Documentation](https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters)
- [sysid "Fix docs for measurement delay and incorrect delay for NEO brushless" PR#125](https://github.com/wpilibsuite/sysid/pull/253), specifically [Lines 58-87](https://github.com/calcmogul/sysid/blob/2bc19fc2cb85a878aa774059474c47bd972cdd8d/sysid-application/src/main/native/include/sysid/analysis/FeedbackControllerPreset.h#L58-L87) of `FeedbackControllerPreset.h `.
- [B.4 "Time Delay Compensation" of Controls Engineering in FRC](https://file.tavsys.net/control/controls-engineering-in-frc.pdf#page=230&zoom=auto,-215,373)
- [FIRST Robotics Competition Documentation on LQR](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#lqr-and-measurement-latency-compensation)
