// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/TalonSRXSimCollection.h>

#include "TalonEncoder.h"

template <class WPI_BaseTalon>
class BaseTalonEncoderSim {
  static_assert(std::is_base_of<ctre::phoenix::motorcontrol::can::BaseTalon,
                                WPI_BaseTalon>::value,
                "Motor type must be a subclass of CTRE's BaseTalon class");

 public:
  /**
   * Constructs from an Encoder object.
   *
   * @param encoder Encoder to simulate
   */
  explicit BaseTalonEncoderSim(BaseTalonEncoder<WPI_BaseTalon>& encoder);

  ctre::phoenix::motorcontrol::TalonSRXSimCollection GetSimCollection();

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
  double GetPeriod() const;

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
  BaseTalonEncoder<WPI_BaseTalon>& m_encoder;
  // In 2021, CTRE Phoenix Libraries only have TalonSRXSimCollection
  ctre::phoenix::motorcontrol::TalonSRXSimCollection m_simCollection;

  // From CTRE documentation.
  int m_defaultSamplesToAverage = 64;
};

template <class WPI_BaseTalon>
BaseTalonEncoderSim<WPI_BaseTalon>::BaseTalonEncoderSim(
    BaseTalonEncoder<WPI_BaseTalon>& encoder)
    : m_encoder(encoder), m_simCollection(encoder.GetMotor()) {}

template <class WPI_BaseTalon>
ctre::phoenix::motorcontrol::TalonSRXSimCollection
BaseTalonEncoderSim<WPI_BaseTalon>::GetSimCollection() {
  return m_simCollection;
}

template <class WPI_BaseTalon>
int BaseTalonEncoderSim<WPI_BaseTalon>::GetCount() const {
  return m_encoder.Get();
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::SetCount(int count) {
  m_simCollection.SetQuadratureRawPosition(count);
}

template <class WPI_BaseTalon>
double BaseTalonEncoderSim<WPI_BaseTalon>::GetPeriod() const {
  return m_encoder.GetPeriod();
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::SetPeriod(double period) {
  // seconds -> distance/second
  // m_distancePerPulse (distance) / period (seconds) = distance / second
  SetRate(GetDistancePerPulse() / period);
}

template <class WPI_BaseTalon>
bool BaseTalonEncoderSim<WPI_BaseTalon>::GetDirection() const {
  return m_encoder.GetDirection();
}

template <class WPI_BaseTalon>
double BaseTalonEncoderSim<WPI_BaseTalon>::GetDistancePerPulse() const {
  return m_encoder.GetDistancePerPulse();
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::SetDistancePerPulse(
    double distancePerPulse) {
  m_encoder.SetDistancePerPulse(distancePerPulse);
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::SetSamplesToAverage(
    int samplesToAverage) {
  m_encoder.SetSamplesToAverage(samplesToAverage);
}

template <class WPI_BaseTalon>
int BaseTalonEncoderSim<WPI_BaseTalon>::GetSamplesToAverage() const {
  return m_encoder.GetSamplesToAverage();
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::ResetData() {
  // This seems to be for the purposes of tests.
  SetDistance(0);
  SetRate(0);
  SetDistancePerPulse(1);
  SetSamplesToAverage(m_defaultSamplesToAverage);
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::SetDistance(double distance) {
  SetCount(distance / GetDistancePerPulse());
}

template <class WPI_BaseTalon>
double BaseTalonEncoderSim<WPI_BaseTalon>::GetDistance() {
  return m_encoder.GetDistance();
}

template <class WPI_BaseTalon>
void BaseTalonEncoderSim<WPI_BaseTalon>::SetRate(double rate) {
  // Convert distance/second into pulses / 100ms
  m_simCollection.SetQuadratureVelocity(rate / (10 * GetDistancePerPulse()));
}

template <class WPI_BaseTalon>
double BaseTalonEncoderSim<WPI_BaseTalon>::GetRate() {
  return m_encoder.GetRate();
}

using TalonSRXEncoderSim =  // NOLINT
    BaseTalonEncoderSim<ctre::phoenix::motorcontrol::can::WPI_TalonSRX>;
using TalonFXEncoderSim =  // NOLINT
    BaseTalonEncoderSim<ctre::phoenix::motorcontrol::can::WPI_TalonFX>;
