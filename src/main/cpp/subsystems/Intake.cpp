#include "Intake.h"

#include <cstdio>

void IntakeSubsystem::Periodic() {
  // send motor info to intake motor
  if (intake_on) {
    // we dont have motors to test the controllers yet so printf will do

    printf("[intake] running motor\n");
  }
}

void IntakeSubsystem::SetIntake(bool intake_set) { intake_on = intake_set; }
