#pragma once

#include "./subsystems/swerve/swerveConstants/Constants.h"

#include <ctre/Phoenix.h>

class CTREConfigs {
public:
	CTREConfigs(Constants* constants);
private:
	TalonFXConfiguration swerveAngleConfig;
	TalonFXConfiguration swerveDriveConfig;
	CANCoderConfiguration swerveCanCoderConfig;
};