// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>
#include <frc/motorcontrol/MotorController.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * Allows multiple MotorController objects to be linked together.
 *
 * A reimplementation of WPILib's MotorControllerGroup using CTRE's CAN
 * follower functionality.
 */
class PhoenixMotorControllerGroup
    : public wpi::Sendable,
      public frc::MotorController,
      public wpi::SendableHelper<PhoenixMotorControllerGroup> {
 public:
  template <class... MotorControllers>
  explicit PhoenixMotorControllerGroup(
      ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
          leadMotorController,
      MotorControllers&... motorControllers);
  explicit PhoenixMotorControllerGroup(
      ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
          leadMotorController,
      std::vector<std::reference_wrapper<
          ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>&&
          motorControllers);

  PhoenixMotorControllerGroup(PhoenixMotorControllerGroup&&) = default;
  PhoenixMotorControllerGroup& operator=(PhoenixMotorControllerGroup&&) =
      default;

  void Set(double speed) override;
  double Get() const override;
  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  void Disable() override;
  void StopMotor() override;

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
      m_leadMotorController;
  std::vector<std::reference_wrapper<
      ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>
      m_motorControllers;

  void Initialize();
};

#include "PhoenixMotorControllerGroup.inc"
