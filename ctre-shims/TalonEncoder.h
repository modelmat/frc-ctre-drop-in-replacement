// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/BaseTalon.h>
#include <frc/Errors.h>
#include <units/time.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * A class to read encoder data from CTRE motors, frc::Encoder compatible.
 */

class TalonEncoder : public wpi::Sendable,
                     public wpi::SendableHelper<TalonEncoder> {
 public:
  explicit TalonEncoder(ctre::phoenix::motorcontrol::can::BaseTalon& motor,
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
  void Reset();

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
  units::second_t GetPeriod() const;

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
   * This sets the multiplier used to determine the distance driven based on
   the
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
   * Initializes the Sendable object
   *
   * Allows this to be seen in LiveWindow and such
   */
  void InitSendable(wpi::SendableBuilder& builder);

  ctre::phoenix::motorcontrol::can::BaseTalon& GetMotor();
  ctre::phoenix::motorcontrol::FeedbackDevice GetSelectedFeedbackSensor() const;

 private:
  ctre::phoenix::motorcontrol::can::BaseTalon& m_motor;
  double m_distancePerPulse = 1;
};
