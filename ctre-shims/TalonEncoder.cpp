// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TalonEncoder.h"

#include <fmt/format.h>

#include <ctre/phoenix/paramEnum.h>
#include <wpi/sendable/SendableRegistry.h>

TalonEncoder::TalonEncoder(ctre::phoenix::motorcontrol::can::BaseTalon& motor,
                           bool reverseDirection)
    : m_motor(motor) {
  SetReverseDirection(reverseDirection);
  wpi::SendableRegistry::AddLW(this, "Talon Encoder", m_motor.GetDeviceID());
}

int TalonEncoder::Get() const {
  return m_motor.GetSelectedSensorPosition();
}

void TalonEncoder::Reset(void) {
  m_motor.SetSelectedSensorPosition(0);
}

units::second_t TalonEncoder::GetPeriod() const {
  // distance / (distance / second) = seconds
  return units::second_t{m_distancePerPulse / GetRate()};
}

double TalonEncoder::GetDistance() const {
  return Get() * m_distancePerPulse;
}

double TalonEncoder::GetRate() const {
  // units per 100 milliseconds to metres per second
  return m_motor.GetSelectedSensorVelocity() * 10 * m_distancePerPulse;
}

bool TalonEncoder::GetDirection() const {
  if (GetRate() >= 0) {
    return true;
  } else {
    return false;
  }
}

void TalonEncoder::SetDistancePerPulse(double distancePerPulse) {
  m_distancePerPulse = distancePerPulse;
}

double TalonEncoder::GetDistancePerPulse() const {
  return m_distancePerPulse;
}

void TalonEncoder::SetReverseDirection(bool reverseDirection) {
  m_motor.SetSensorPhase(reverseDirection);
}

void TalonEncoder::SetSamplesToAverage(int samplesToAverage) {
  // (n & (n-1)) checks if it is a power of two. See:
  // http://
  // www.graphics.stanford.edu/~seander/bithacks.html#DetermineIfPowerOf2
  if (samplesToAverage < 1 || samplesToAverage > 64 ||
      (samplesToAverage & (samplesToAverage - 1)) != 0) {
    throw FRC_MakeError(
        frc::err::ParameterOutOfRange,
        "Samples to average must be a power of 2 between 1 and 64, got {}",
        samplesToAverage);
  }
  m_motor.ConfigVelocityMeasurementWindow(samplesToAverage);
}

int TalonEncoder::GetSamplesToAverage() const {
  return m_motor.ConfigGetParameter(
      ctre::phoenix::ParamEnum::eSampleVelocityWindow, 0);
}

ctre::phoenix::motorcontrol::can::BaseTalon& TalonEncoder::GetMotor() {
  return m_motor;
}

ctre::phoenix::motorcontrol::FeedbackDevice
TalonEncoder::GetSelectedFeedbackSensor() const {
  return static_cast<ctre::phoenix::motorcontrol::FeedbackDevice>(
      m_motor.ConfigGetParameter(ctre::phoenix::ParamEnum::eFeedbackSensorType,
                                 0));
}

void TalonEncoder::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType(
      fmt::format("Encoder ({})",
                  ctre::phoenix::motorcontrol::FeedbackDeviceRoutines::toString(
                      GetSelectedFeedbackSensor())));
  builder.AddDoubleProperty(
      "Speed", [=] { return GetRate(); }, nullptr);
  builder.AddDoubleProperty(
      "Distance", [=] { return GetDistance(); }, nullptr);
  builder.AddDoubleProperty(
      "Distance per Tick", [=] { return GetDistancePerPulse(); }, nullptr);
}
