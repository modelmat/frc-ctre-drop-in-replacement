// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PhoenixSpeedControllerGroup.h"

#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableRegistry.h>

// Can't use a delegated constructor here because of an MSVC bug.
// https://developercommunity.visualstudio.com/content/problem/583/compiler-bug-with-delegating-a-constructor.html

PhoenixSpeedControllerGroup::PhoenixSpeedControllerGroup(
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
        leadSpeedController,
    std::vector<std::reference_wrapper<
        ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>&&
        speedControllers)
    : m_leadSpeedController(leadSpeedController),
      m_speedControllers(std::move(speedControllers)) {
  Initialize();
}

void PhoenixSpeedControllerGroup::Initialize() {
  frc::SendableRegistry::GetInstance().AddChild(this, &m_leadSpeedController);
  for (auto& speedController : m_speedControllers) {
    frc::SendableRegistry::GetInstance().AddChild(this, &speedController.get());
    speedController.get().Follow(m_leadSpeedController);
    speedController.get().SetInverted(
        ctre::phoenix::motorcontrol::InvertType::FollowMaster);
  }
  static int instances = 0;
  ++instances;
  frc::SendableRegistry::GetInstance().Add(this, "PhoenixSpeedControllerGroup",
                                           instances);
}

void PhoenixSpeedControllerGroup::Set(double speed) {
  m_leadSpeedController.Set(m_isInverted ? -speed : speed);
}

double PhoenixSpeedControllerGroup::Get() const {
  return m_leadSpeedController.Get() * (m_isInverted ? -1 : 1);
}

void PhoenixSpeedControllerGroup::SetInverted(bool isInverted) {
  m_isInverted = isInverted;
}

bool PhoenixSpeedControllerGroup::GetInverted() const {
  return m_isInverted;
}

void PhoenixSpeedControllerGroup::Disable() {
  m_leadSpeedController.Disable();
  for (auto speedController : m_speedControllers) {
    speedController.get().Disable();
  }
}

void PhoenixSpeedControllerGroup::StopMotor() {
  m_leadSpeedController.StopMotor();
  for (auto speedController : m_speedControllers) {
    speedController.get().StopMotor();
  }
}

void PhoenixSpeedControllerGroup::PIDWrite(double output) {
  Set(output);
}

void PhoenixSpeedControllerGroup::InitSendable(frc::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Speed Controller");
  builder.SetActuator(true);
  builder.SetSafeState([=]() { StopMotor(); });
  builder.AddDoubleProperty(
      "Value", [=]() { return Get(); }, [=](double value) { Set(value); });
}
