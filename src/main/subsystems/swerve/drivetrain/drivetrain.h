// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "./subsystems/swerve/swerveConstants/Constants.h"
#include "./subsystems/swerve/swerveModule/SwerveModule.h"

#include <AHRS.h>
#include <units/velocity.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <vector>

class Drivetrain : public frc2::SubsystemBase {
public:
  Drivetrain();
  void drive(frc::Translation2d translation, units::radians_per_second_t rotation, bool fieldRelative, bool isOpenLoop);
  void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
  frc::Pose2d getPose();
  void resetOdomentry(frc::Pose2d pose);
  wpi::array<frc::SwerveModuleState*, 4> getStates();
  void zeroGyro();
  frc::Rotation2d getYaw();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  Constants constants;
  frc::SwerveDriveOdometry<4>* swerveOdometry;
  std::vector<SwerveModule*> swerveModules{ 4 };
  AHRS navx{ frc::SPI::Port::kMXP };
};
