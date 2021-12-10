// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/TalonFXSimCollection.h>
#include <ctre/phoenix/motorcontrol/TalonSRXSimCollection.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/Errors.h>

#include "TalonEncoder.h"

template <class TalonSimCollection>
class BaseTalonEncoderSim {
  static_assert(
      std::is_same_v<ctre::phoenix::motorcontrol::TalonSRXSimCollection,
                     TalonSimCollection> ||
          std::is_same_v<ctre::phoenix::motorcontrol::TalonFXSimCollection,
                         TalonSimCollection>,
      "Sim collection must be a CTRE sim collection class.");

 public:
  /**
   * Constructs from an Encoder object.
   *
   * @param encoder Encoder to simulate
   */
  explicit BaseTalonEncoderSim(TalonEncoder& encoder);

  // frc::sim::EncoderSim functions

  /**
   * Read the count of the encoder.
   *
   * @return the count
   */
  int GetCount() const;

  /**
   * Change the count of the encoder.
   *
   * @param count the new count
   */
  void SetCount(int count);

  /**
   * Read the period of the encoder.
   *
   * @return the encoder period
   */
  units::second_t GetPeriod() const;

  /**
   * Change the encoder period.
   *
   * @param period the new period
   */
  void SetPeriod(double period);

  // These are no-op in WPILib
  // bool GetReset() const;
  // void SetReset(bool reset);

  /**
   * The last direction the encoder value changed.
   *
   * @return The last direction the encoder value changed.
   */
  bool GetDirection() const;

  /**
   * Read the distance per pulse of the encoder.
   *
   * @return the encoder distance per pulse
   */
  double GetDistancePerPulse() const;

  /**
   * Change the encoder distance per pulse.
   *
   * @param distancePerPulse the new distance per pulse
   */
  void SetDistancePerPulse(double distancePerPulse);

  /**
   * Set the Samples to Average which specifies the number of samples of the
   * timer to average when calculating the period.
   *
   * Perform averaging to account for mechanical imperfections or as
   * oversampling to increase resolution.
   *
   * Defaults to 64. See also, CTRE's ConfigVelocityMeasurementPeriod().
   * https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
   * See also, the note in Measurement-Delays.md in the root of this
   repository.
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
   * See also, the note in Measurement-Delays.md in the root of this
   repository.
   *
   * @return The number of samples being averaged (one of 1, 2, 4, 8, 16, 32,
   *         or 64).
   */
  int GetSamplesToAverage() const;

  /**
   * Resets all simulation data for this encoder.
   */
  void ResetData();

  /**
   * Change the encoder distance.
   *
   * @param distance the new distance
   */
  void SetDistance(double distance);

  /**
   * Read the distance of the encoder.
   *
   * @return the encoder distance
   */
  double GetDistance();

  /**
   * Change the rate of the encoder.
   *
   * @param rate the new rate
   */
  void SetRate(double rate);

  /**
   * Get the rate of the encoder.
   *
   * @return the rate of change
   */
  double GetRate();

 private:
  TalonEncoder& m_encoder;
  TalonSimCollection m_simCollection;
};

template <class TalonSimCollection>
BaseTalonEncoderSim<TalonSimCollection>::BaseTalonEncoderSim(
    TalonEncoder& encoder)
    : m_encoder(encoder), m_simCollection(encoder.GetMotor()) {}

template <class TalonSimCollection>
int BaseTalonEncoderSim<TalonSimCollection>::GetCount() const {
  return m_encoder.Get();
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::SetCount(int count) {
  if constexpr (std::is_same_v<
                    TalonSimCollection,
                    ctre::phoenix::motorcontrol::TalonSRXSimCollection>) {
    ctre::phoenix::motorcontrol::FeedbackDevice selected =
        m_encoder.GetSelectedFeedbackSensor();

    switch (selected) {
      // Also handles CTRE_MagEncoder_Relative as the same enum value
      case ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder:
        m_simCollection.SetQuadratureRawPosition(count);
        break;

      case ctre::phoenix::motorcontrol::FeedbackDevice::Analog:
        m_simCollection.SetAnalogPosition(count);
        break;

      default:
        throw FRC_MakeError(
            frc::err::IncompatibleMode,
            "Selected feedback sensor is not supported: {}",
            ctre::phoenix::motorcontrol::FeedbackDeviceRoutines::toString(
                selected));
        break;
    }
  } else {  // Only TalonSRX & TalonFX exist.
    // TalonFX only has one sensor.
    m_simCollection.SetIntegratedSensorRawPosition(count);
  }
}

template <class TalonSimCollection>
units::second_t BaseTalonEncoderSim<TalonSimCollection>::GetPeriod() const {
  return m_encoder.GetPeriod();
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::SetPeriod(double period) {
  // seconds -> distance/second
  // m_distancePerPulse (distance) / period (seconds) = distance / second
  SetRate(GetDistancePerPulse() / period);
}

template <class TalonSimCollection>
bool BaseTalonEncoderSim<TalonSimCollection>::GetDirection() const {
  return m_encoder.GetDirection();
}

template <class TalonSimCollection>
double BaseTalonEncoderSim<TalonSimCollection>::GetDistancePerPulse() const {
  return m_encoder.GetDistancePerPulse();
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::SetDistancePerPulse(
    double distancePerPulse) {
  m_encoder.SetDistancePerPulse(distancePerPulse);
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::SetSamplesToAverage(
    int samplesToAverage) {
  m_encoder.SetSamplesToAverage(samplesToAverage);
}

template <class TalonSimCollection>
int BaseTalonEncoderSim<TalonSimCollection>::GetSamplesToAverage() const {
  return m_encoder.GetSamplesToAverage();
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::ResetData() {
  // This seems to be for the purposes of tests.
  SetDistance(0);
  SetRate(0);
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::SetDistance(double distance) {
  SetCount(distance / GetDistancePerPulse());
}

template <class TalonSimCollection>
double BaseTalonEncoderSim<TalonSimCollection>::GetDistance() {
  return m_encoder.GetDistance();
}

template <class TalonSimCollection>
void BaseTalonEncoderSim<TalonSimCollection>::SetRate(double rate) {
  // Convert distance/second into pulses / 100ms
  double rateInNativeUnits = rate / (10 * GetDistancePerPulse());

  if constexpr (std::is_same_v<
                    TalonSimCollection,
                    ctre::phoenix::motorcontrol::TalonSRXSimCollection>) {
    ctre::phoenix::motorcontrol::FeedbackDevice selected =
        m_encoder.GetSelectedFeedbackSensor();

    switch (selected) {
      // Also handles CTRE_MagEncoder_Relative as the same enum value
      case ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder:
        m_simCollection.SetQuadratureVelocity(rateInNativeUnits);
        break;

      case ctre::phoenix::motorcontrol::FeedbackDevice::Analog:
        m_simCollection.SetAnalogVelocity(rateInNativeUnits);
        break;

      default:
        throw FRC_MakeError(
            frc::err::IncompatibleMode,
            "Selected feedback sensor is not supported: {}",
            ctre::phoenix::motorcontrol::FeedbackDeviceRoutines::toString(
                selected));
        break;
    }
  } else {  // Only TalonSRX & TalonFX exist.
    // TalonFX only has one sensor.
    m_simCollection.SetIntegratedSensorVelocity(rateInNativeUnits);
  }
}

template <class TalonSimCollection>
double BaseTalonEncoderSim<TalonSimCollection>::GetRate() {
  return m_encoder.GetRate();
}

using TalonSRXEncoderSim =  // NOLINT
    BaseTalonEncoderSim<ctre::phoenix::motorcontrol::TalonSRXSimCollection>;
using TalonFXEncoderSim =  // NOLINT
    BaseTalonEncoderSim<ctre::phoenix::motorcontrol::TalonFXSimCollection>;
