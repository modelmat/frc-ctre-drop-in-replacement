# Shim for CTRE Smart Motor Controllers to act like WPILib-native objects 

This repository contains various unofficial shims around CTRE's motor controllers that are designed to allow for *as close to possible*, drop in replacements within WPILib examples / documentation code.
Where possible, these shims incorporate sane units.

## Supported Classes

Listed below are classes which have shims.
Currently, there are no plans for any other classes.

PID and Motion Profiles cannot work as "drop-in" replacements, since the existing API provides `Calculate()` which is then `Set()` outside of the class; doing a `Set()` stops the motor controller's PID motion, and thus this would require code modification. 
A shim which uses better units could work, but is out of scope (as it would not be a WPILib-style object).

## PhoenixSpeedControllerGroup

This is a shim around the `SpeedControllerGroup` class ([C++](https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1SpeedControllerGroup.html), [Java](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/SpeedController.html)) which uses the `Follow()` method to reduce CANBus usage.
The original `SpeedControllerGroup` could be used with the `WPI_*` motors of CTRE, but this runs `Set()` on every motor controller in the group.

It should be noted that `motor.SetInverted(InvertType::FollowMaster)` is set within `PhoenixSpeedControllerGroup`. If you need differing behaviour, you must set `motor.SetInverted()` somewhere in your code to ensure behaviour as intended ([C++](https://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#a93095f6262bc3a6e30bb7f346c526b95), [Java](https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#ace2b403c71a6560981e4a70f33e18889)).

**All** of the functionality of the WPILib has been replicated (same class hierarchy, all of the same methods).
It is compatible with **all** of the CTRE motor controllers (specifically, those which inherit from `WPI_BaseMotorController`).

To use it, `#include` or `import` the header file, and change the class used to instantiate the speed controller group to `PhoenixSpeedControllerGroup`.

This should be something like this ([C++ full example](https://github.com/modelmat/frc-ctre-drop-in-replacement/commit/27d8250a39452f988f4e707767b22388920489c0?diff=unified#diff-d2abd75f7c28d73dba1fed7d0e49aaec9ca945a95a61fe4f9819657389d62e12L123-R132)), omitting the CTRE namespace for clarity:

```diff
- #include <frc/SpeedControllerGroup.h>
+ #include "PhoenixSpeedControllerGroup.h"
...
- frc::PWMSparkMax m_left1{DriveConstants::kLeftMotor1Port};
- frc::PWMSparkMax m_left2{DriveConstants::kLeftMotor2Port};
- frc::PWMSparkMax m_right1{DriveConstants::kRightMotor1Port};
- frc::PWMSparkMax m_right2{DriveConstants::kRightMotor2Port};
+ WPI_TalonSRX     m_left1{DriveConstants::kLeftMotor1Port};
+ WPI_VictorSPX    m_left2{DriveConstants::kLeftMotor2Port};
+ WPI_TalonSRX     m_right1{DriveConstants::kRightMotor1Port
+ WPI_VictorSPX    m_right2{DriveConstants::kRightMotor2Port};
...
- frc::SpeedControllerGroup m_leftMotors{m_left1, m_left2};
- frc::SpeedControllerGroup m_rightMotors{m_right1, m_right2};
+ PhoenixSpeedControllerGroup m_leftMotors{m_left1, m_left2};
+ PhoenixSpeedControllerGroup m_rightMotors{m_right1, m_right2};
```

## TalonEncoder
This is a shim around the `Encoder` class ([C++](https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1Encoder.html), [Java](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Encoder.html)) which brings sane unit support and WPILib-style API, replacing the need to use `motor.GetSelectedSensor{Position,Velocity}()` from CTRE's API.

Specifically, the class names are `TalonSRXEncoder` and `TalonFXEncoder`.

**Most** of the class functionality has been replicated with some minor exceptions (inheritance from `CounterBase`, and member functions `SetMinRate()`, `SetMaxPeriod()`, `GetStopped()`, `GetRaw()`, `GetEncodingScale()`, `SetIndexSource()`, `SetSimDevice()`, and `GetFPGAIndex()`).
The class works with only Talon motor controllers, as only they can have encoders (spefically, classes which inherit from `BaseTalon` **AND** `WPI_BaseMotorController`).

It should also be noted that CTRE's default values for the *sample period* (`100 ms`) and *samples to average*/*window size* (`64`) cause a measurement delay.
For more detail, see `Measurement-Delays.md` in the repository root.


This class does not run `ConfigSelectedFeedbackSensor()` ([C++](https://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#a36fd80e529de4282dafdddd8d04cccf3), [Java](https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#ab5de4b3115af274126f802cdb910244a)), so if you are using a connected encoder which is not Quadrature (TalonSRX) or Integrated (TalonFX) it will be necessary to run this yourself.
It is *recommended* to do so as well, to ensure that the motor controller is using the correct settings; also a good idea is `ConfigFactoryDefault()` for each motor. 
This ensures that each motor's settings are always correctly set, even if they get swapped around between different robots.

It is necessary to slightly modify the class instantiation, as `TalonEncoder` is constructed using the motor as the argument, rather than DigitalInput ports on the RoboRIO.
The changes should be something like this: ([full C++ example](https://github.com/modelmat/frc-ctre-drop-in-replacement/commit/83d39a52e376a335e05962f872c13ad5749353ba#diff-d2abd75f7c28d73dba1fed7d0e49aaec9ca945a95a61fe4f9819657389d62e12L138-L143)), using the motor controllers shown earlier. 

```diff
- #include <frc/Encoder.h>
+ #include "TalonEncoder.h"
...
- frc::Encoder m_leftEncoder{DriveConstants::kLeftEncoderPorts[0],
-                            DriveConstants::kLeftEncoderPorts[1]};
- frc::Encoder m_rightEncoder{DriveConstants::kRightEncoderPorts[0],
-                             DriveConstants::kRightEncoderPorts[1]};
+ TalonSRXEncoder m_leftEncoder{m_left1};
+ TalonSRXEncoder m_rightEncoder{m_right1};
```

## TalonEncoderSim

This is a shim around the `EncoderSim` class ([C++](https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1EncoderSim.html), [Java](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/EncoderSim.html)) which can be used with the `TalonEncoder` class.

Specifically, the class names are `TalonSRXEncoderSim` and `TalonFXEncoderSim`.

```diff
- #include <frc/sim/EncoderSim.h>
+ #include "TalonEncoderSim.h"
...
- frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
- frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
+ TalonSRXEncoderSim m_leftEncoderSim{m_leftEncoder};
+ TalonSRXEncoderSim m_rightEncoderSim{m_rightEncoder};
```