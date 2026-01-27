
#pragma once

#include "frc2/command/SubsystemBase.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem() {}
  void Periodic() override;

  void SetIntake(bool intake_set);

 private:
  // motors here
  bool intake_on = false;
};
