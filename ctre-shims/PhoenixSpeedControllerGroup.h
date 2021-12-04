// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>
#include <frc/SpeedController.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableHelper.h>

/**
 * Allows multiple SpeedController objects to be linked together.
 *
 * A reimplementation of WPILib's SpeedControllerGroup using CTRE's CAN
 * follower functionality.
 */
class PhoenixSpeedControllerGroup
    : public frc::Sendable,
      public frc::SpeedController,
      public frc::SendableHelper<PhoenixSpeedControllerGroup> {
 public:
  template <class... SpeedControllers>
  explicit PhoenixSpeedControllerGroup(
      ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
          leadSpeedController,
      SpeedControllers&... speedControllers);
  explicit PhoenixSpeedControllerGroup(
      ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
          leadSpeedController,
      std::vector<std::reference_wrapper<
          ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>&&
          speedControllers);

  PhoenixSpeedControllerGroup(PhoenixSpeedControllerGroup&&) = default;
  PhoenixSpeedControllerGroup& operator=(PhoenixSpeedControllerGroup&&) =
      default;

  void Set(double speed) override;
  double Get() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  void Disable() override;
  void StopMotor() override;
  void PIDWrite(double output) override;

  void InitSendable(frc::SendableBuilder& builder) override;

 private:
  bool m_isInverted = false;
  ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
      m_leadSpeedController;
  std::vector<std::reference_wrapper<
      ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>
      m_speedControllers;

  void Initialize();
};

#include "PhoenixSpeedControllerGroup.inc"
