// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/swerve/drivetrain/Drivetrain.h"

#include <wpi/array.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>

class SwerveAuto
	: public frc2::CommandHelper<frc2::SequentialCommandGroup,
	SwerveAuto> {
public:
	SwerveAuto(Drivetrain* swerve);
private:
	Drivetrain* drivetrain;
};
