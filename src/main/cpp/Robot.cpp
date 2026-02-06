// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <cstdio>
#include <utility>

#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

Robot::Robot() {
  // Initialize all of your commands and subsystems here

  m_chooser.AddOption("No Auto", AUTO_NOTHING);
  m_chooser.AddOption("Blue Default", AUTO_BLUE_DEFAULT);
  m_chooser.AddOption("Red Default", AUTO_RED_DEFAULT);

  frc::SmartDashboard::PutData("Auto Selection", &m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driveController.GetY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driveController.GetX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driveController.GetZ(), OIConstants::kDriveDeadband)},
            true, m_slowMode);
      },
      {&m_drive}));
}

void Robot::ConfigureButtonBindings() {
  // button to stop being pushed
  frc2::JoystickButton(&m_driveController, 6)  // button 6 on joystick?
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
  //
  // slow mode
  frc2::JoystickButton(&m_driveController, 1)  // trigger
      .ToggleOnTrue(
          new frc2::InstantCommand([this]() { m_slowMode = true; }, {}));

  frc2::JoystickButton(&m_driveController, 1)  // trigger
      .ToggleOnFalse(
          new frc2::InstantCommand([this]() { m_slowMode = false; }, {}));

  // activate intake
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kA)
      .ToggleOnTrue(new frc2::InstantCommand(
          [this] { m_intake.SetIntake(true); }, {&m_intake}));
  frc2::JoystickButton(&m_actionController, frc::XboxController::Button::kA)
      .ToggleOnFalse(new frc2::InstantCommand(
          [this] { m_intake.SetIntake(false); }, {&m_intake}));
}

frc2::CommandPtr Robot::GetAutonomousCommand(){
    //had to delete the other method for auto cause it broke
    return PathPlannerAuto("Example Auto").ToPtr();
}
