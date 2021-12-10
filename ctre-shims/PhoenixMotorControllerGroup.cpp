// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PhoenixMotorControllerGroup.h"

#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

// Can't use a delegated constructor here because of an MSVC bug.
// https://developercommunity.visualstudio.com/content/problem/583/compiler-bug-with-delegating-a-constructor.html

PhoenixMotorControllerGroup::PhoenixMotorControllerGroup(
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
        leadMotorController,
    std::vector<std::reference_wrapper<
        ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>&&
        motorControllers)
    : m_leadMotorController(leadMotorController),
      m_motorControllers(std::move(motorControllers)) {
  Initialize();
}

void PhoenixMotorControllerGroup::Initialize() {
  wpi::SendableRegistry::AddChild(this, &m_leadMotorController);
  for (auto& motorController : m_motorControllers) {
    wpi::SendableRegistry::AddChild(this, &motorController.get());
    motorController.get().Follow(m_leadMotorController);
    motorController.get().SetInverted(
        ctre::phoenix::motorcontrol::InvertType::FollowMaster);
  }
  static int instances = 0;
  ++instances;
  wpi::SendableRegistry::Add(this, "PhoenixMotorControllerGroup", instances);
}

void PhoenixMotorControllerGroup::Set(double speed) {
  m_leadMotorController.Set(speed);
}

double PhoenixMotorControllerGroup::Get() const {
  return m_leadMotorController.Get();
}

void PhoenixMotorControllerGroup::SetInverted(bool isInverted) {
  m_leadMotorController.SetInverted(isInverted);
}

bool PhoenixMotorControllerGroup::GetInverted() const {
  return m_leadMotorController.GetInverted();
}

void PhoenixMotorControllerGroup::Disable() {
  m_leadMotorController.Disable();
  for (auto motorController : m_motorControllers) {
    motorController.get().Disable();
  }
}

void PhoenixMotorControllerGroup::StopMotor() {
  m_leadMotorController.StopMotor();
  for (auto motorController : m_motorControllers) {
    motorController.get().StopMotor();
  }
}

void PhoenixMotorControllerGroup::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Phoenix Motor Controller");
  builder.SetActuator(true);
  builder.SetSafeState([=]() { StopMotor(); });
  builder.AddDoubleProperty(
      "Value", [=]() { return Get(); }, [=](double value) { Set(value); });
}
