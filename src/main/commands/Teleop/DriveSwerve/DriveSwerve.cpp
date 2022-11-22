// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveSwerve.h"

DriveSwerve::DriveSwerve(Drivetrain* swerve, frc::Joystick* controller, int translationAxis, int strafeAxis, int rotationAxis, bool fieldRelative, bool openLoop) : swerve(swerve), controller(controller), translationAxis(translationAxis), strafeAxis(strafeAxis), rotationAxis(rotationAxis), fieldRelative(fieldRelative), openLoop(openLoop) {
	// Use addRequirements() here to declare subsystem dependencies.
	this->swerve = swerve;
	AddRequirements(swerve);

	this->controller = controller;
	this->translationAxis = translationAxis;
	this->strafeAxis = strafeAxis;
	this->rotationAxis = rotationAxis;
	this->fieldRelative = fieldRelative;
	this->openLoop = openLoop;
}

// Called when the command is initially scheduled.
void DriveSwerve::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveSwerve::Execute() {
	double yAxis = -controller->GetRawAxis(translationAxis);
	double xAxis = controller->GetRawAxis(strafeAxis);
	double rAxis = controller->GetRawAxis(rotationAxis);

	yAxis = (abs(yAxis) < swerve->constants.stickDeadBand) ? 0 : yAxis;
	xAxis = (abs(xAxis) < swerve->constants.stickDeadBand) ? 0 : xAxis;
	rAxis = (abs(rAxis) < swerve->constants.stickDeadBand) ? 0 : rAxis;

	units::meter_t y{ yAxis };
	units::meter_t x{ xAxis };
	units::radian_t r{ rAxis };


	double speed = units::unit_cast<double>(swerve->constants.swerve.maxSpeed);
	frc::Translation2d tempTranslation{ x,y };
	translation = tempTranslation * speed;
	rotation = rAxis * swerve->constants.swerve.maxAngularVelocity;
	swerve->drive(translation, rotation, fieldRelative, openLoop);
}

// Called once the command ends or is interrupted.
void DriveSwerve::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveSwerve::IsFinished() {
	return false;
}
