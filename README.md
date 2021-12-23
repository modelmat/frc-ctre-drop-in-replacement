# Shim for CTRE Smart Motor Controllers to act like WPILib-native objects

This repository contains various shims around CTRE's motor controllers that are designed to allow for *as close to possible*, drop in replacements within WPILib examples / documentation code.
Where possible, these shims incorporate sane units.

For the 2022 C++ version, see [the cpp branch](https://github.com/modelmat/frc-ctre-drop-in-replacement/tree/cpp).
For the 2022 Java version, see [the java branch](https://github.com/modelmat/frc-ctre-drop-in-replacement/tree/java).
For the 2021 C++ version of the code, see [the 2021 branch](https://github.com/modelmat/frc-ctre-drop-in-replacement/tree/2021).

## "Installing" the Shim Library

To install the Java version of the library, place of a copy of the `src/main/java/ctre_shims/` folder in the same location in your WPILib robot project.
Unlike C++, this requires no modification to `build.gradle`.

## Supported Classes

Listed below are classes which have shims.
Currently, there are no plans for any other classes.

PID and Motion Profiles cannot work as "drop-in" replacements, since the existing API provides `calculate()` which is then `set()` outside of the class; doing a `set()` stops the motor controller's PID motion, and thus this would require code modification.
A shim which uses better units could work, but is out of scope (as it would not be a WPILib-style object).

## PhoenixMotorControllerGroup

This is a shim around the [`MotorControllerGroup`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/MotorController.html) class which uses the `follow()` method to reduce CANBus usage.
The original `MotorControllerGroup` could be used with the `WPI_*` motors of CTRE, but this runs `set()` on every motor controller in the group, which increase CANbus utilisation.

It should be noted that `motor.setInverted(InvertType::FollowMaster)` is set within `PhoenixMotorControllerGroup`. If you need differing behaviour, you must set [`motor.setInverted()`](https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#ace2b403c71a6560981e4a70f33e18889) somewhere in your code after class construction to ensure behaviour as intended.
**All** of the functionality of the WPILib has been replicated (same class hierarchy, all of the same methods).
It is compatible with **all** of the CTRE motor controllers (specifically, those which inherit from `WPI_BaseMotorController`).

To use it, `import` the class file, and change the class used to instantiate the speed controller group to `PhoenixMotorControllerGroup`.

This should be something like this, omitting the CTRE namespace for clarity:

```diff
- import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
+ import ctre_shims.PhoenixMotorControllerGroup;
...
- private final MotorControllerGroup m_leftMotors =
-     new MotorControllerGroup(m_left1, m_left2));
- private final MotorControllerGroup m_rightMotors =
-     new MotorControllerGroup(m_right1, m_right2));
+ private final PhoenixMotorControllerGroup m_leftMotors =
+     new PhoenixMotorControllerGroup(m_left1, m_left2);
+ private final PhoenixMotorControllerGroup m_rightMotors =
+     new PhoenixMotorControllerGroup(m_right1, m_right2);
```

## TalonEncoder
This is a shim around the [`Encoder`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Encoder.html) class which brings sane unit support and WPILib-style API, replacing the need to use `motor.getSelectedSensor{Position,Velocity}()` from CTRE's API.

**Most** of the class functionality has been replicated with some minor exceptions (inheritance from `CounterBase`, and member functions `setMinRate()`, `setMaxPeriod()`, `getStopped()`, `getRaw()`, `getEncodingScale()`, `setIndexSource()`, `setSimDevice()`, and `getFPGAIndex()`).
The class works with only Talon motor controllers, as only they can have encoders (specifically, classes which inherit from CTRE's `BaseTalon`).

It should also be noted that CTRE's default values for the *sample period* (`100 ms`) and *samples to average*/*window size* (`64`) cause a measurement delay.
For more detail, see `Measurement-Delays.md` in the repository root.

This class does not call [`configSelectedFeedbackSensor()`](https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#ab5de4b3115af274126f802cdb910244a), so if you are using a connected encoder which is not Quadrature (TalonSRX) or Integrated (TalonFX) it will be necessary to call this yourself.
It is *recommended* to do so as well, to ensure that the motor controller is using the correct settings; also a good idea is `configFactoryDefault()` for each motor.
This ensures that each motor's settings are always correctly set, even if they get swapped around between different robots.

It is necessary to slightly modify the class instantiation, as `TalonEncoder` is constructed using the motor as the argument, rather than DigitalInput ports on the RoboRIO.
The changes should be something like this, using the motor controllers shown earlier:

```diff
- import edu.wpi.first.wpilibj.Encoder;
+ import ctre_shims.TalonEncoder;
...
- private final Encoder m_leftEncoder =
-     new Encoder(
-         DriveConstants.kLeftEncoderPorts[0],
-         DriveConstants.kLeftEncoderPorts[1],
-         DriveConstants.kLeftEncoderReversed);
- private final Encoder m_rightEncoder =
-     new Encoder(
-         DriveConstants.kRightEncoderPorts[0],
-         DriveConstants.kRightEncoderPorts[1],
-         DriveConstants.kRightEncoderReversed);
+ private final TalonEncoder m_leftEncoder =
+     new TalonEncoder(m_leftMotor1, DriveConstants.kLeftEncoderReversed);
+ private final TalonEncoder m_rightEncoder =
+     new TalonEncoder(m_rightMotor1, DriveConstants.kRightEncoderReversed);
```

## TalonEncoderSim

This is a shim for the [`EncoderSim`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/EncoderSim.html) class which can be used with the `TalonEncoder` class.

```diff
- import edu.wpi.first.wpilibj.simulation.EncoderSim;
+ import ctre_shims.TalonEncoderSim;
...
-  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
-  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
+  private TalonEncoderSim m_leftEncoderSim = new TalonEncoderSim(m_leftEncoder);
+  private TalonEncoderSim m_rightEncoderSim = new TalonEncoderSim(m_rightEncoder);
```
