// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/FRCUsageReporting.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <cstdio>

#include "../Constants.h"
#include "../LimelightHelpers.h"

using namespace DriveConstants;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
  // Do all subsystem initialization here
  // ...

    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
      // Load the RobotConfig from the GUI settings. You should probably
      // store this in your Constants file
      RobotConfig config = RobotConfig::fromGUISettings();
      // Configure the AutoBuilder last
      AutoBuilder::configure(
          [this]() { return GetPose(); },  // Robot pose supplier
          [this](frc::Pose2d pose) {
            ResetOdometry(pose);
          },  // Method to reset odometry (will be called if your auto has a
              // starting pose)
          [this]() {
            return GetRobotRelativeSpeeds();
          },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          [this](auto speeds, auto feedforwards) {
            driveRobotRelative(speeds);
          },  // Method that will drive the robot given ROBOT RELATIVE
              // ChassisSpeeds. Also optionally outputs individual module
              // feedforwards
          std::make_shared<
              PPHolonomicDriveController>(  // PPHolonomicController
                                            // is the built in path
                                            // following controller
                                            // for holonomic drive
                                            // trains
              PIDConstants(5.0, 0.0, 0.0),  // Translation PID constants
              PIDConstants(5.0, 0.0, 0.0)   // Rotation PID constants
              ),
          config,  // The robot configuration
          []() {
            // Boolean supplier that controls when the path will be mirrored for
            // the red alliance This will flip the path being followed to the
            // red side of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = DriverStation::GetAlliance();
            if (alliance) {
              return alliance.value() == DriverStation::Alliance::kRed;
            }
            return false;
          },
          this  // Reference to this subsystem to set requirements
      );
      // Usage reporting for MAXSwerve template
      HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
                 HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

      frc::SmartDashboard::PutData("Field", &m_field);
    }

    void DriveSubsystem::Periodic() {
      units::degree_t robotYaw = m_gyro.GetAngle(m_gyro.GetYawAxis());
      m_odometry.SetVisionMeasurementStdDevs({0.5, 0.5, 9999999.0});
      LimelightHelpers::SetRobotOrientation("", robotYaw.value(), 0.0, 0.0, 0.0,
                                            0.0, 0.0);

      // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html
      // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
      LimelightHelpers::PoseEstimate limelightMeasurement =
          LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

      m_odometry.AddVisionMeasurement(limelightMeasurement.pose,
                                      limelightMeasurement.timestampSeconds);

      LimelightHelpers::PoseEstimate limelightFancyMeasurement =
          LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(
              "limelight-fancy");

      m_odometry.AddVisionMeasurement(
          limelightFancyMeasurement.pose,
          limelightFancyMeasurement.timestampSeconds);

      m_odometry.Update(
          frc::Rotation2d(robotYaw),
          {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
           m_frontRight.GetPosition(), m_rearRight.GetPosition()});
      m_field.SetRobotPose(m_odometry.GetEstimatedPosition());
    }

    void DriveSubsystem::Drive(
        units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
        units::radians_per_second_t rot, bool fieldRelative, bool slowMode) {
      // Convert the commanded speeds into the correct units for the drivetrain
      units::meters_per_second_t speedUsed =
          slowMode ? DriveConstants::kSlowModeSpeed : DriveConstants::kMaxSpeed;

      units::meters_per_second_t xSpeedDelivered = xSpeed.value() * speedUsed;
      units::meters_per_second_t ySpeedDelivered = ySpeed.value() * speedUsed;
      units::radians_per_second_t rotDelivered =
          rot.value() * (slowMode ? DriveConstants::kSlowModeAngularSpeed
                                  : DriveConstants::kMaxAngularSpeed);

      auto states = kDriveKinematics.ToSwerveModuleStates(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeedDelivered, ySpeedDelivered, rotDelivered,
                              frc::Rotation2d(units::radian_t{m_gyro.GetAngle(
                                  frc::ADIS16470_IMU::IMUAxis::kZ)}))
                        : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered,
                                             rotDelivered});

      kDriveKinematics.DesaturateWheelSpeeds(&states,
                                             DriveConstants::kMaxSpeed);

      auto [fl, fr, bl, br] = states;

      m_frontLeft.SetDesiredState(fl);
      m_frontRight.SetDesiredState(fr);
      m_rearLeft.SetDesiredState(bl);
      m_rearRight.SetDesiredState(br);
    }

    void DriveSubsystem::SetX() {
      m_frontLeft.SetDesiredState(
          frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
      m_frontRight.SetDesiredState(
          frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
      m_rearLeft.SetDesiredState(
          frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
      m_rearRight.SetDesiredState(
          frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
    }

    void DriveSubsystem::SetModuleStates(
        wpi::array<frc::SwerveModuleState, 4> desiredStates) {
      kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                             DriveConstants::kMaxSpeed);
      m_frontLeft.SetDesiredState(desiredStates[0]);
      m_frontRight.SetDesiredState(desiredStates[1]);
      m_rearLeft.SetDesiredState(desiredStates[2]);
      m_rearRight.SetDesiredState(desiredStates[3]);
    }

    void DriveSubsystem::ResetEncoders() {
      m_frontLeft.ResetEncoders();
      m_rearLeft.ResetEncoders();
      m_frontRight.ResetEncoders();
      m_rearRight.ResetEncoders();
    }

    units::degree_t DriveSubsystem::GetHeading() const {
      return frc::Rotation2d(units::radian_t{m_gyro.GetAngle(
                                 frc::ADIS16470_IMU::IMUAxis::kZ)})
          .Degrees();
    }

    void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }

    double DriveSubsystem::GetTurnRate() {
      return -m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
    }

    frc::Pose2d DriveSubsystem::GetPose() {
      return m_odometry.GetEstimatedPosition();
    }

    void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
      m_odometry.ResetPosition(
          GetHeading(),
          {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
           m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
          pose);
    }
}
