// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/BaseTalon.h>
#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/PIDSource.h>
#include <frc/WPIErrors.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc/smartdashboard/SendableRegistry.h>

/**
 * A class to read encoder data from CTRE motors, frc::Encoder compatible.
 */

template <class WPI_BaseTalon>
class BaseTalonEncoder
    : public frc::PIDSource,
      public frc::Sendable,
      public frc::SendableHelper<BaseTalonEncoder<WPI_BaseTalon>> {
  static_assert(std::is_base_of<ctre::phoenix::motorcontrol::can::BaseTalon,
                                WPI_BaseTalon>::value,
                "Motor type must be a subclass of CTRE's BaseTalon class");

 public:
  explicit BaseTalonEncoder(WPI_BaseTalon& motor,
                            bool reverseDirection = false);

  /**
   * Gets the current count.
   *
   * Returns the current count on the Encoder.
   *
   * @return Current count from the Encoder.
   */
  int Get() const;

  /**
   * Reset the Encoder distance to zero.
   *
   * Resets the current count to zero on the encoder.
   */
  void Reset(void);

  /**
   * Returns the period of the most recent pulse.
   *
   * Returns the period of the most recent Encoder pulse in seconds. This method
   * compensates for the decoding type.
   *
   * Warning: This returns unscaled periods. Use GetRate() for rates that are
   * scaled using the value from SetDistancePerPulse().
   *
   * @return Period in seconds of the most recent pulse.
   */
  double GetPeriod() const;

  /**
   * Get the distance the robot has driven since the last reset.
   *
   * @return The distance driven since the last reset as scaled by the value
   *         from SetDistancePerPulse().
   */
  double GetDistance() const;

  /**
   * Get the current rate of the encoder.
   *
   * Units are distance per second as scaled by the value from
   * SetDistancePerPulse().
   *
   * @return The current rate of the encoder.
   */
  double GetRate() const;

  /**
   * The last direction the encoder value changed.
   *
   * @return The last direction the encoder value changed.
   */
  bool GetDirection() const;

  /**
   * Set the distance per pulse for this encoder.
   *
   * This sets the multiplier used to determine the distance driven based on the
   * count value from the encoder.
   *
   * Do not include the decoding type in this scale.  The library already
   * compensates for the decoding type.
   *
   * Set this value based on the encoder's rated Pulses per Revolution and
   * factor in gearing reductions following the encoder shaft.
   *
   * This distance can be in any units you like, linear or angular.
   *
   * @param distancePerPulse The scale factor that will be used to convert
   *                         pulses to useful units.
   */
  void SetDistancePerPulse(double distancePerPulse);

  /**
   * Get the distance per pulse for this encoder.
   *
   * @return The scale factor that will be used to convert pulses to useful
   *         units.
   */
  double GetDistancePerPulse() const;

  /**
   * Set the direction sensing for this encoder.
   *
   * This sets the direction sensing on the encoder so that it could count in
   * the correct software direction regardless of the mounting.
   *
   * @param reverseDirection true if the encoder direction should be reversed
   */
  void SetReverseDirection(bool reverseDirection);

  /**
   * Set the Samples to Average which specifies the number of samples of the
   * timer to average when calculating the period.
   *
   * Perform averaging to account for mechanical imperfections or as
   * oversampling to increase resolution.
   *
   * Defaults to 64. See also, CTRE's ConfigVelocityMeasurementPeriod().
   * https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
   * See also, the note in Measurement-Delays.md in the root of this repository.
   *
   * @param samplesToAverage The number of samples to average, one of
   *                         1, 2, 4, 8, 16, 32, or 64.
   */
  void SetSamplesToAverage(int samplesToAverage);

  /**
   * Get the Samples to Average which specifies the number of samples of the
   * timer to average when calculating the period.
   *
   * Perform averaging to account for mechanical imperfections or as
   * oversampling to increase resolution.
   *
   * Defaults to 64. See also, CTRE's ConfigVelocityMeasurementPeriod().
   * https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
   * See also, the note in Measurement-Delays.md in the root of this repository.
   *
   * @return The number of samples being averaged (one of 1, 2, 4, 8, 16, 32,
   *         or 64).
   */
  int GetSamplesToAverage() const;

  // For frc::PIDSource
  double PIDGet() override;

  /**
   * Initializes the Sendable object
   *
   * Allows this to be seen in LiveWindow and such
   */
  void InitSendable(frc::SendableBuilder& builder);

  WPI_BaseTalon& GetMotor();

 private:
  WPI_BaseTalon& m_motor;
  double m_distancePerPulse = 1;
};

template <class WPI_BaseTalon>
BaseTalonEncoder<WPI_BaseTalon>::BaseTalonEncoder(WPI_BaseTalon& motor,
                                                  bool reverseDirection)
    : m_motor(motor) {
  SetReverseDirection(reverseDirection);
  frc::SendableRegistry::GetInstance().AddLW(this, "Talon Encoder",
                                             m_motor.GetDeviceID());
}

template <class WPI_BaseTalon>
int BaseTalonEncoder<WPI_BaseTalon>::Get() const {
  return m_motor.GetSelectedSensorPosition();
}

template <class WPI_BaseTalon>
void BaseTalonEncoder<WPI_BaseTalon>::Reset(void) {
  m_motor.SetSelectedSensorPosition(0);
}

template <class WPI_BaseTalon>
double BaseTalonEncoder<WPI_BaseTalon>::GetPeriod() const {
  // distance / (distance / second) = seconds
  return m_distancePerPulse / GetRate();
}

template <class WPI_BaseTalon>
double BaseTalonEncoder<WPI_BaseTalon>::GetDistance() const {
  return Get() * m_distancePerPulse;
}

template <class WPI_BaseTalon>
double BaseTalonEncoder<WPI_BaseTalon>::GetRate() const {
  // units per 100 milliseconds to metres per second
  return m_motor.GetSelectedSensorVelocity() * 10 * m_distancePerPulse;
}

template <class WPI_BaseTalon>
bool BaseTalonEncoder<WPI_BaseTalon>::GetDirection() const {
  if (GetRate() >= 0) {
    return true;
  } else {
    return false;
  }
}

template <class WPI_BaseTalon>
void BaseTalonEncoder<WPI_BaseTalon>::SetDistancePerPulse(
    double distancePerPulse) {
  m_distancePerPulse = distancePerPulse;
}

template <class WPI_BaseTalon>
double BaseTalonEncoder<WPI_BaseTalon>::GetDistancePerPulse() const {
  return m_distancePerPulse;
}

template <class WPI_BaseTalon>
void BaseTalonEncoder<WPI_BaseTalon>::SetReverseDirection(
    bool reverseDirection) {
  // TODO: Confirm this works? Not sure if it
  // should.
  m_motor.SetSensorPhase(reverseDirection);
}

template <class WPI_BaseTalon>
void BaseTalonEncoder<WPI_BaseTalon>::SetSamplesToAverage(
    int samplesToAverage) {
  // (n & (n-1)) checks if it is a power of two. See:
  // http://www.graphics.stanford.edu/~seander/bithacks.html#DetermineIfPowerOf2
  if (samplesToAverage < 1 || samplesToAverage > 64 ||
      (samplesToAverage & (samplesToAverage - 1)) != 0) {
    wpi_setWPIErrorWithContext(
        ParameterOutOfRange,
        "Samples to average must be a power of 2 between 1 and 64.");
    return;
  }
  m_motor.ConfigVelocityMeasurementWindow(samplesToAverage);
}

template <class WPI_BaseTalon>
int BaseTalonEncoder<WPI_BaseTalon>::GetSamplesToAverage() const {
  ctre::phoenix::motorcontrol::can::BaseTalonConfiguration& config;
  m_motor.GetAllConfigs(config);
  return config.velocityMeasurementWindow();
}

template <class WPI_BaseTalon>
double BaseTalonEncoder<WPI_BaseTalon>::PIDGet() {
  switch (GetPIDSourceType()) {
    case frc::PIDSourceType::kDisplacement:
      return GetDistance();
    case frc::PIDSourceType::kRate:
      return GetRate();
    default:
      return 0.0;
  }
}

template <class WPI_BaseTalon>
WPI_BaseTalon& BaseTalonEncoder<WPI_BaseTalon>::GetMotor() {
  return m_motor;
}

template <class WPI_BaseTalon>
void BaseTalonEncoder<WPI_BaseTalon>::InitSendable(
    frc::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Quadrature Encoder");
  builder.AddDoubleProperty(
      "Speed", [=]() { return GetRate(); }, nullptr);
  builder.AddDoubleProperty(
      "Distance", [=]() { return GetDistance(); }, nullptr);
  builder.AddDoubleProperty(
      "Distance per Tick", [=]() { return GetDistancePerPulse(); }, nullptr);
}

using TalonSRXEncoder =  // NOLINT
    BaseTalonEncoder<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>;
using TalonFXEncoder =  // NOLINT
    BaseTalonEncoder<ctre::phoenix::motorcontrol::can::WPI_TalonFX>;
