// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include "PhoenixMotorControllerGroup.h"

template <class... MotorControllers>
inline PhoenixMotorControllerGroup::PhoenixMotorControllerGroup(
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
        leadMotorController,
    MotorControllers&... motorControllers)
    : m_leadMotorController(leadMotorController),
      m_motorControllers(
          std::vector<std::reference_wrapper<
              ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>{
              motorControllers...}) {
  Initialize();
}
