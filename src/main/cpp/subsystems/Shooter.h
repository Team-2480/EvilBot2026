
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

    // JULIA: any new motor you add will need to be configurated here
    // this is the constructor so any code here will be run as soon as the
    // object is instanced (ie the robot starts)
  }
  void Periodic() override;

  void SetShooter(bool shooter_set);
  void SetShooterMode(ShooterMode mode);

 private:
  bool shooter_on = false;

  ShooterMode mode = SHOOTER_NONE;

  // falcon 500
  ctre::phoenix6::hardware::TalonFX shooter_driver{20};
  // JULIA: we will need another motor so make a new
  // ctre::phoenix6::hardware::TalonFX
  //
  // refer to the bible when picking IDs
  // https://docs.google.com/document/d/1VkR9zvviwuhPBft1adSYg7TGN60f-zLs2Nebqqzaj-k/edit?usp=sharing

  ctre::phoenix6::controls::VelocityVoltage shooter_driver_speed =
      ctre::phoenix6::controls::VelocityVoltage{5_tps}.WithSlot(0);

  ctre::phoenix6::controls::VelocityVoltage stop_speed =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

  // JULIA: this is the info we give to a ctre::phoenix6::hardware::TalonFX to
  // set the speed of the motor
  //
  // so _tps here is turns per second, to set postion we need _tr ie turns
  //
  // WithPosition is a function to update the position later on so not during
  // intialization (intialization is here) we should have a SetShooterRot() or
  // something function that will update the rot
  ctre::phoenix6::controls::PositionDutyCycle turn_shooter =
      ctre::phoenix6::controls::PositionDutyCycle{0.2_tps}.WithPosition();
};
