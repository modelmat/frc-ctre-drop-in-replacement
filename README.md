# Shim for CTRE Smart Motor Controllers to act like WPILib-native objects

This repository contains various shims around CTRE's motor controllers that are designed to allow for *as close to possible*, drop in replacements within WPILib examples / documentation code.
Where possible, these shims incorporate sane units.

For the 2022 C++ version, see [the cpp branch](https://github.com/modelmat/frc-ctre-drop-in-replacement/tree/cpp).
For the 2022 Java version, see [the java branch](https://github.com/modelmat/frc-ctre-drop-in-replacement/tree/java).
For the 2021 C++ version of the code, see [the 2021 branch](https://github.com/modelmat/frc-ctre-drop-in-replacement/tree/2021).

## "Installing" the Shim Library

To install the C++ version of the library, place a copy of the `ctre-shims` folder in the root directory of a WPILib robot project.
Then, in `build.gradle`, modify the `sources.cpp` block as below:

```diff
             sources.cpp {
                 source {
-                    srcDir 'src/main/cpp'
+                    srcDirs 'src/main/cpp', 'ctre-shims'
                     include '**/*.cpp', '**/*.cc'
                 }
                 exportedHeaders {
-                    srcDir 'src/main/include'
+                    srcDirs 'src/main/include', 'ctre-shims'
                 }
             }
```

## Supported Classes

Listed below are classes which have shims.
Currently, there are no plans for any other classes.

PID and Motion Profiles cannot work as "drop-in" replacements, since the existing API provides `Calculate()` which is then `Set()` outside of the class; doing a `Set()` stops the motor controller's PID motion, and thus this would require code modification.
A shim which uses better units could work, but is out of scope (as it would not be a WPILib-style object).

## PhoenixMotorControllerGroup

This is a shim around the [`MotorControllerGroup`](https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1MotorControllerGroup.html) class which uses the `Follow()` method to reduce CANBus usage.
The original `MotorControllerGroup` could be used with the `WPI_*` motors of CTRE, but this runs `Set()` on every motor controller in the group, which increase CANbus utilisation.

It should be noted that `motor.SetInverted(InvertType::FollowMaster)` is set within `PhoenixMotorControllerGroup`. If you need differing behaviour, you must set [`motor.SetInverted()`](https://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#a93095f6262bc3a6e30bb7f346c526b95) somewhere in your code after class construction to ensure behaviour as intended.

**All** of the functionality of the WPILib has been replicated (same class hierarchy, all of the same methods).
It is compatible with **all** of the CTRE motor controllers (specifically, those which inherit from `WPI_BaseMotorController`).

To use it, `#include` the header file, and change the class used to instantiate the speed controller group to `PhoenixMotorControllerGroup`.

This should be something like this, omitting the CTRE namespace for clarity:

```diff
- #include <frc/MotorControllerGroup.h>
+ #include "PhoenixMotorControllerGroup.h"
...
- frc::MotorControllerGroup m_leftMotors{m_left1, m_left2};
- frc::MotorControllerGroup m_rightMotors{m_right1, m_right2};
+ PhoenixMotorControllerGroup m_leftMotors{m_left1, m_left2};
+ PhoenixMotorControllerGroup m_rightMotors{m_right1, m_right2};
```

## TalonEncoder
This is a shim around the [`Encoder`](https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1Encoder.html) class which brings sane unit support and WPILib-style API, replacing the need to use `motor.GetSelectedSensor{Position,Velocity}()` from CTRE's API.

**Most** of the class functionality has been replicated with some minor exceptions (inheritance from `CounterBase`, and member functions `SetMinRate()`, `SetMaxPeriod()`, `GetStopped()`, `GetRaw()`, `GetEncodingScale()`, `SetIndexSource()`, `SetSimDevice()`, and `GetFPGAIndex()`).
The class works with only Talon motor controllers, as only they can have encoders (specifically, classes which inherit from CTRE's `BaseTalon`).

It should also be noted that CTRE's default values for the *sample period* (`100 ms`) and *samples to average*/*window size* (`64`) cause a measurement delay.
For more detail, see `Measurement-Delays.md` in the repository root.

This class does not call [`ConfigSelectedFeedbackSensor()`](https://www.ctr-electronics.com/downloads/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_base_motor_controller.html#a36fd80e529de4282dafdddd8d04cccf3), so if you are using a connected encoder which is not Quadrature (TalonSRX) or Integrated (TalonFX) it will be necessary to call this yourself.
It is *recommended* to do so as well, to ensure that the motor controller is using the correct settings; also a good idea is `ConfigFactoryDefault()` for each motor.
This ensures that each motor's settings are always correctly set, even if they get swapped around between different robots.

It is necessary to slightly modify the class instantiation, as `TalonEncoder` is constructed using the motor as the argument, rather than DigitalInput ports on the RoboRIO.
The changes should be something like this, using the motor controllers shown earlier:

```diff
- #include <frc/Encoder.h>
+ #include "TalonEncoder.h"
...
- frc::Encoder m_leftEncoder{DriveConstants::kLeftEncoderPorts[0],
-                            DriveConstants::kLeftEncoderPorts[1],
-                            DriveConstants::kLeftEncoderReversed};
- frc::Encoder m_rightEncoder{DriveConstants::kRightEncoderPorts[0],
-                             DriveConstants::kRightEncoderPorts[1],
-                             DriveConstants::kRightEncoderReversed};
+ TalonEncoder m_leftEncoder{m_left1, DriveConstants::kLeftEncoderReversed};
+ TalonEncoder m_rightEncoder{m_right1, DriveConstants::kRightEncoderReversed};
```

## TalonEncoderSim

This is a shim for the [`EncoderSim`](https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classfrc_1_1sim_1_1EncoderSim.html) class which can be used with the `TalonEncoder` class.

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
