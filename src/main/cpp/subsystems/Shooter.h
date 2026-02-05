
#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "frc2/command/SubsystemBase.h"

enum ShooterMode {
  SHOOTER_NONE,
  SHOOTER_MANUAL,
  SHOOTER_AUTO,
};

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem() {
    ctre::phoenix6::configs::TalonFXConfiguration cfg;

    cfg.Slot0.kP = 0.1;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;

    cfg.MotorOutput.Inverted =
        ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

    shooter_driver.GetConfigurator().Apply(cfg);

    shooter_driver.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
  void Periodic() override;

  void SetShooter(bool shooter_set);
  void SetShooterMode(ShooterMode mode);

 private:
  bool shooter_on = false;

  ShooterMode mode = SHOOTER_NONE;

  // falcon 500
  ctre::phoenix6::hardware::TalonFX shooter_driver{20};

  ctre::phoenix6::controls::VelocityVoltage shooter_driver_speed =
      ctre::phoenix6::controls::VelocityVoltage{5_tps}.WithSlot(0);

  ctre::phoenix6::controls::VelocityVoltage stop_speed =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

  ctre::phoenix6::controls::PositionDutyCycle turn_shooter = 
    ctre::phoenix6::controls::PositionDutyCycle{0.2_tps}.WithPosition();
};
