// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here
	bool isFieldOriented = true;
	bool openLoop = true;
	swerve.SetDefaultCommand(DriveSwerve(&swerve, &driver, translationAxis, strafeAxis, rotationAxis, isFieldOriented, openLoop));

	// Configure the button bindings
	ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
	// Configure your button bindings here

	zeroGyro.WhenPressed(frc2::InstantCommand([this] { swerve.zeroGyro(); }, {}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
	// An example command will be run in autonomous
	// return &m_autonomousCommand;
	return nullptr;
}
