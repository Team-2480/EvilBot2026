#include "Shooter.h"

void ShooterSubsystem::Periodic() {
  switch (mode) {
    case SHOOTER_AUTO:
      break;
    case SHOOTER_MANUAL:
      break;
    case SHOOTER_NONE:
      break;
  }

  if (shooter_on) {
    shooter_driver.SetControl(shooter_driver_speed);
    shooter_driver.SetControl(turn_shooter);
  } else {
    shooter_driver.SetControl(stop_speed);
  }
}

void ShooterSubsystem::SetShooter(bool shooter_set) {
  shooter_on = shooter_set;
}

void ShooterSubsystem::SetShooterMode(ShooterMode mode_set) { mode = mode_set; }
