
#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkMaxConfig.h>

#include "../Regulator.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "frc2/command/SubsystemBase.h"
#include "rev/config/SparkMaxConfigAccessor.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem() {
    intake_driver.Configure(
        IntakeUpDownConfig(),
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    intake_up_down_driver.Configure(
        IntakeDriverConfig(),
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters);

    up_down_regulator.SetTargets(4.5, -1);
    up_down_regulator.SetRatio(60);
  }
  void Periodic() override;

  void SetIntake(bool intake_set);

  enum IntakeUpDown { INTAKE_UP, INTAKE_DOWN };
  void SetIntakeUpDown(IntakeUpDown intake_up_down);

 private:
  // motors here
  bool intake_on = false;
  IntakeUpDown intake_up_down = INTAKE_UP;

  rev::spark::SparkMax intake_driver =
      rev::spark::SparkMax(40, rev::spark::SparkMax::MotorType::kBrushless);
  rev::spark::SparkClosedLoopController intake_driver_controller =
      intake_driver.GetClosedLoopController();

  ctre::phoenix6::controls::VelocityVoltage intake_driver_speed =
      ctre::phoenix6::controls::VelocityVoltage{5_tps}.WithSlot(0);

  ctre::phoenix6::controls::VelocityVoltage stop_speed =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

  rev::spark::SparkMax intake_up_down_driver =
      rev::spark::SparkMax(41, rev::spark::SparkMax::MotorType::kBrushless);

  rev::spark::SparkClosedLoopController intake_up_down_controller =
      intake_up_down_driver.GetClosedLoopController();

  rev::spark::SparkRelativeEncoder intake_up_down_encoder =
      intake_up_down_driver.GetEncoder();

  MotorRegulator up_down_regulator =
      MotorRegulator(&intake_up_down_driver, &intake_up_down_controller);

  static rev::spark::SparkMaxConfig &IntakeUpDownConfig() {
    static rev::spark::SparkMaxConfig direct_config{};

    direct_config.encoder.PositionConversionFactor(1).VelocityConversionFactor(
        1);
    direct_config.closedLoop
        .SetFeedbackSensor(
            rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(0.9, 0, 0)
        .VelocityFF(0.0)
        .OutputRange(-1, 1)
        .maxMotion.MaxVelocity(4500)
        .MaxAcceleration(6000)
        .AllowedClosedLoopError(1);

    // direct_config.softLimit.ReverseSoftLimitEnabled(true)
    //     .ReverseSoftLimit(-1 * 60)
    //     .ForwardSoftLimitEnabled(true)
    //     .ForwardSoftLimit(7 * 60);

    return direct_config;
  }

  static rev::spark::SparkMaxConfig &IntakeDriverConfig() {
    static rev::spark::SparkMaxConfig direct_config{};

    direct_config.encoder.PositionConversionFactor(1).VelocityConversionFactor(
        1);
    direct_config.closedLoop
        .SetFeedbackSensor(
            rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(0.9, 0, 0)
        .VelocityFF(0.0)
        .OutputRange(-1, 1)
        .maxMotion.MaxVelocity(4500)
        .MaxAcceleration(6000)
        .AllowedClosedLoopError(1);

    // direct_config.softLimit.ReverseSoftLimitEnabled(true)
    //     .ReverseSoftLimit(-1 * 60)
    //     .ForwardSoftLimitEnabled(true)
    //     .ForwardSoftLimit(7 * 60);

    return direct_config;
  }
};
