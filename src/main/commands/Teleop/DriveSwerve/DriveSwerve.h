// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/swerve/drivetrain/Drivetrain.h"
#include "subsystems/swerve/swerveConstants/Constants.h"

#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <frc/Joystick.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveSwerve : public frc2::CommandHelper<frc2::CommandBase, DriveSwerve> {
public:
	DriveSwerve(Drivetrain* swerve, frc::Joystick* controller, int translationAxis, int strafeAxis, int rotationAxis, bool fieldRelative, bool openLoop);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

private:
	units::radians_per_second_t rotation;
	frc::Translation2d translation;
	Drivetrain* swerve;
	frc::Joystick* controller;
	int translationAxis;
	int strafeAxis;
	int rotationAxis;
	bool fieldRelative;
	bool openLoop;
};
