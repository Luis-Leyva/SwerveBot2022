// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain.h"

Drivetrain::Drivetrain() {

	zeroGyro();

	swerveOdometry = new frc::SwerveDriveOdometry<4>(constants.swerve.swerveKinematics, getYaw());
	swerveModules[0] = new SwerveModule(&constants, 0, &constants.swerve.Mod0);
	swerveModules[1] = new SwerveModule(&constants, 1, &constants.swerve.Mod1);
	swerveModules[2] = new SwerveModule(&constants, 2, &constants.swerve.Mod2);
	swerveModules[3] = new SwerveModule(&constants, 3, &constants.swerve.Mod3);
}

void Drivetrain::drive(frc::Translation2d translation, units::radians_per_second_t rotation, bool fieldRelative, bool isOpenLoop) {
	units::meters_per_second_t xAxis{ units::unit_cast<double>(translation.X()) };
	units::meters_per_second_t yAxis{ units::unit_cast<double>(translation.Y()) };

	wpi::array<frc::SwerveModuleState, 4> swerveModuleStates = constants.swerve.swerveKinematics.ToSwerveModuleStates(
		fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
			xAxis,
			yAxis,
			rotation,
			getYaw())
		: frc::ChassisSpeeds{
			xAxis,
			yAxis,
			rotation });
	frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&swerveModuleStates, constants.swerve.maxSpeed);

	for (int i = 0; i < 4; i++) {
		swerveModules[i]->setDesiredState(swerveModuleStates[swerveModules[i]->getModuleNumber()], isOpenLoop);
	}
}

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, 4>  desiredStates) {
	frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&desiredStates, constants.swerve.maxSpeed);

	for (int i = 0; i < 4; i++) {
		swerveModules[i]->setDesiredState(desiredStates[swerveModules[i]->getModuleNumber()], false);
	}
}

frc::Pose2d Drivetrain::getPose() {
	return swerveOdometry->GetPose();
}

void Drivetrain::resetOdometry(frc::Pose2d pose) {
	swerveOdometry->ResetPosition(pose, getYaw());
}

wpi::array<frc::SwerveModuleState, 4> Drivetrain::getStates() {
	wpi::array<frc::SwerveModuleState, 4> states{ swerveModules[0]->getState(), swerveModules[1]->getState(), swerveModules[2]->getState(), swerveModules[3]->getState() };
	return states;
}

void Drivetrain::zeroGyro() {
	navx.ZeroYaw();
}

frc::Rotation2d Drivetrain::getYaw() {
	if (navx.IsMagnetometerCalibrated()) {
		frc::Rotation2d yaw(units::degree_t(navx.GetFusedHeading()));
		return yaw;
	}

	frc::Rotation2d yaw(units::degree_t(360.0 - navx.GetYaw()));
	return yaw;
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {
	wpi::array<frc::SwerveModuleState, 4> states{ getStates() };
	swerveOdometry->Update(getYaw(), states[0], states[1], states[2], states[3]);


	frc::SmartDashboard::PutNumber("0 Cancoder", units::unit_cast<double>(swerveModules[0]->getCanCoder().Degrees()));
	frc::SmartDashboard::PutNumber("0 Integrated", units::unit_cast<double>(swerveModules[0]->getState().angle.Degrees()));
	frc::SmartDashboard::PutNumber("0 Velocity", units::unit_cast<double>(swerveModules[0]->getState().speed));
	frc::SmartDashboard::PutNumber("0 Target Angle", units::unit_cast<double>(swerveModules[0]->getLastDesiredState().angle.Degrees()));

	frc::SmartDashboard::PutNumber("1 Cancoder", units::unit_cast<double>(swerveModules[1]->getCanCoder().Degrees()));
	frc::SmartDashboard::PutNumber("1 Integrated", units::unit_cast<double>(swerveModules[1]->getState().angle.Degrees()));
	frc::SmartDashboard::PutNumber("1 Velocity", units::unit_cast<double>(swerveModules[1]->getState().speed));
	frc::SmartDashboard::PutNumber("1 Target Angle", units::unit_cast<double>(swerveModules[1]->getLastDesiredState().angle.Degrees()));

	frc::SmartDashboard::PutNumber("2 Cancoder", units::unit_cast<double>(swerveModules[2]->getCanCoder().Degrees()));
	frc::SmartDashboard::PutNumber("2 Integrated", units::unit_cast<double>(swerveModules[2]->getState().angle.Degrees()));
	frc::SmartDashboard::PutNumber("2 Velocity", units::unit_cast<double>(swerveModules[2]->getState().speed));
	frc::SmartDashboard::PutNumber("2 Target Angle", units::unit_cast<double>(swerveModules[2]->getLastDesiredState().angle.Degrees()));

	frc::SmartDashboard::PutNumber("3 Cancoder", units::unit_cast<double>(swerveModules[3]->getCanCoder().Degrees()));
	frc::SmartDashboard::PutNumber("3 Integrated", units::unit_cast<double>(swerveModules[3]->getState().angle.Degrees()));
	frc::SmartDashboard::PutNumber("3 Velocity", units::unit_cast<double>(swerveModules[3]->getState().speed));
	frc::SmartDashboard::PutNumber("3 Target Angle", units::unit_cast<double>(swerveModules[3]->getLastDesiredState().angle.Degrees()));
}