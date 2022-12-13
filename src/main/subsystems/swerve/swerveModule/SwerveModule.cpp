// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

SwerveModule::SwerveModule(Constants* constants, int moduleNumber, SwerveModuleConstants* moduleConstants) {
	this->moduleNumber = moduleNumber;
	this->constants = constants;

	angleOffset = moduleConstants->angleOffset;

	constants->swerve.configure();

	rotatorPID.EnableContinuousInput(0, 360);

	/* Angle Encoder Config */
	angleEncoder = new CANCoder(moduleConstants->canCoderID);
	configAngleEncoder();

	/* Angle Motor Config */
	angleMotor = new TalonFX(moduleConstants->angleMotorID);
	configAngleMotor();

	/* Drive Motor Config */
	driveMotor = new TalonFX(moduleConstants->driveMotorID);
	configDriveMotor();

	lastAngle = getState().angle.Degrees();
}

void SwerveModule::setDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop) {
	desiredState = CTREModuleState::Optimize(desiredState, getState().angle); // Custom optimize command, since default WPILib optimize assumes // continuous controller which CTRE is // not
	lastDesiredState = desiredState;
	if (isOpenLoop) {
		double percentOutput = desiredState.speed / constants->swerve.maxSpeed;
		driveMotor->Set(ControlMode::PercentOutput, percentOutput);
	} else {
		double velocity = Conversions::MPSToFalcon(desiredState.speed,
			constants->swerve.wheelCircumference, constants->swerve.driveGearRatio);
		driveMotor->Set(ControlMode::Velocity, velocity, DemandType::DemandType_ArbitraryFeedForward,
			units::unit_cast<double>(feedforward.Calculate(desiredState.speed)));
	}

	units::degree_t angle = (units::math::abs(desiredState.speed) <= (constants->swerve.maxSpeed * 0.01)) ? lastAngle
		: desiredState.angle.Degrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.
	angleMotor->Set(ControlMode::Position, units::unit_cast<double>(angle));
	lastAngle = angle;
}

void SwerveModule::resetToAbsolute() {
	double absolutePosition = units::unit_cast<double>(getCanCoder().Degrees() - angleOffset);
	angleMotor->SetSelectedSensorPosition(absolutePosition);
}

void SwerveModule::configAngleEncoder() {
	angleEncoder->ConfigFactoryDefault();
	angleEncoder->ConfigAllSettings(constants->swerve.swerveCanCoderConfig);
}

void SwerveModule::configAngleMotor() {
	angleMotor->ConfigFactoryDefault();
	angleMotor->ConfigAllSettings(constants->swerve.swerveAngleConfig);
	angleMotor->SetInverted(constants->swerve.angleMotorInvert);
	angleMotor->SetNeutralMode(constants->swerve.angleNeutralMode);
	resetToAbsolute();
}

void SwerveModule::configDriveMotor() {
	driveMotor->ConfigFactoryDefault();
	driveMotor->ConfigAllSettings(constants->swerve.swerveDriveConfig);
	driveMotor->SetInverted(constants->swerve.driveMotorInvert);
	driveMotor->SetNeutralMode(constants->swerve.driveNeutralMode);
	driveMotor->SetSelectedSensorPosition(0);
}

frc::Rotation2d SwerveModule::getCanCoder() {
	return frc::Rotation2d(units::degree_t(angleEncoder->GetAbsolutePosition())).RotateBy(units::degree_t(angleOffset));
}

frc::SwerveModuleState SwerveModule::getState() {
	units::meters_per_second_t velocity = Conversions::falconToMPS(driveMotor->GetSelectedSensorVelocity(),
		constants->swerve.wheelCircumference, constants->swerve.driveGearRatio);
	frc::Rotation2d angle = getCanCoder();
	return frc::SwerveModuleState{ velocity, angle };
}

int SwerveModule::getModuleNumber() {
	return moduleNumber;
}

frc::SwerveModuleState SwerveModule::getLastDesiredState() {
	return lastDesiredState;
}
