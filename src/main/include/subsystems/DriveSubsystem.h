// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/ADXRS450_GyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>

#include "Constants.h"
#include "PhoenixMotorControllerGroup.h"
#include "TalonEncoder.h"
#include "TalonEncoderSim.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically during simulation.
   */
  void SimulationPeriodic() override;

  // Subsystem methods go here.

  units::ampere_t GetCurrentDraw() const;

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

  /**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  double GetAverageEncoderDistance();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // The motor controllers
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left1{
      DriveConstants::kLeftMotor1Port};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_left2{
      DriveConstants::kLeftMotor2Port};
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right1{
      DriveConstants::kRightMotor1Port};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_right2{
      DriveConstants::kRightMotor2Port};

  // The motors on the left side of the drive
  PhoenixMotorControllerGroup m_leftMotors{m_left1, m_left2};

  // The motors on the right side of the drive
  PhoenixMotorControllerGroup m_rightMotors{m_right1, m_right2};

  // The robot's drive
  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  // The left-side drive encoder
  TalonEncoder m_leftEncoder{m_left1, DriveConstants::kLeftEncoderReversed};

  // The right-side drive encoder
  TalonEncoder m_rightEncoder{m_right1, DriveConstants::kRightEncoderReversed};

  // The gyro sensor
  frc::ADXRS450_Gyro m_gyro;

  // Odometry class for tracking robot pose
  frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

  // These classes help simulate our drivetrain.
  frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
      DriveConstants::kDrivetrainPlant,
      DriveConstants::kTrackwidth,
      DriveConstants::kDrivetrainGearbox,
      DriveConstants::kDrivetrainGearing,
      DriveConstants::kWheelDiameter / 2,
      {0.001, 0.001, 0.0001, 0.1, 0.1, 0.005, 0.005}};

  TalonSRXEncoderSim m_leftEncoderSim{m_leftEncoder};
  TalonSRXEncoderSim m_rightEncoderSim{m_rightEncoder};
  frc::sim::ADXRS450_GyroSim m_gyroSim{m_gyro};

  // The Field2d class shows the field in the sim GUI.
  frc::Field2d m_fieldSim;
};
