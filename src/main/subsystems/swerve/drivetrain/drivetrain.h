// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "./subsystems/swerve/swerveConstants/Constants.h"

#include <AHRS.h>

class drivetrain {
public:
	drivetrain(Constants* constants);
private:
	Constants* constants;
	AHRS ahrs{ frc::SPI::Port::kMXP };
};
