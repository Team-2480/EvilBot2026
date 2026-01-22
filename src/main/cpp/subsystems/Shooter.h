
#pragma once

#include "frc2/command/SubsystemBase.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem() {}
  void Periodic() override;

  void SetShooter(bool shooter_set);

 private:
  bool shooter_on = false;
};
