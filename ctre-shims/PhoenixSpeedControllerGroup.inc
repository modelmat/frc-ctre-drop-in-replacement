// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include "PhoenixSpeedControllerGroup.h"

template <class... SpeedControllers>
inline PhoenixSpeedControllerGroup::PhoenixSpeedControllerGroup(
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController&
        leadSpeedController,
    SpeedControllers&... speedControllers)
    : m_leadSpeedController(leadSpeedController),
      m_speedControllers(
          std::vector<std::reference_wrapper<
              ctre::phoenix::motorcontrol::can::WPI_BaseMotorController>>{
              speedControllers...}) {
  Initialize();
}
