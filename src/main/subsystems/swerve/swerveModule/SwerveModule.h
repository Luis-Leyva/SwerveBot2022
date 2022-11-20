// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "./subsystems/swerve/swerveConstants/Constants.h"
#include "./util/SwerveModuleConstants.h"
#include "./util/CTREModuleState.h"
#include "./math/Conversions.h"

#include <Math.h>
#include <ctre/Phoenix.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/units.h>

class SwerveModule {
public:
	SwerveModule(Constants* constants, int moduleNumber, SwerveModuleConstants moduleConstants);
	void setDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop);
	void resetToAbsolute();
	void configAngleEncoder();
	void configAngleMotor();
	void configDriveMotor();
	frc::Rotation2d getCanCoder();
	frc::SwerveModuleState* getState();
	int getModuleNumber();
	frc::SwerveModuleState getLastDesiredState();
private:
	Constants* constants;
	int moduleNumber;
	double angleOffset;
	TalonFX* angleMotor;
	TalonFX* driveMotor;
	CANCoder* angleEncoder;
	units::degree_t lastAngle;
	frc::SwerveModuleState lastDesiredState;
	frc::SimpleMotorFeedforward<units::meters> feedforward{ constants->swerve.driveKS, constants->swerve.driveKV, constants->swerve.driveKA };
};
