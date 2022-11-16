#include "CTREConfigs.h"

CTREConfigs::CTREConfigs(Constants* constants) {
	// Swerve Angle Motor Configuration
	SupplyCurrentLimitConfiguration angleSupplyLimit{
		constants->swerve.angleEnableCurrentLimit,
		constants->swerve.angleContinuousCurrentLimit,
		constants->swerve.anglePeakCurrentLimit,
		constants->swerve.anglePeakCurrentDuration
	};

	swerveAngleConfig.slot0.kP = constants->swerve.angleKP;
	swerveAngleConfig.slot0.kI = constants->swerve.angleKI;
	swerveAngleConfig.slot0.kD = constants->swerve.angleKD;
	swerveAngleConfig.slot0.kF = constants->swerve.angleKF;
	swerveAngleConfig.supplyCurrLimit = angleSupplyLimit;
	swerveAngleConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;

	// Swerve Drive Motor Configuration
	SupplyCurrentLimitConfiguration driveSupplyLimit{
		constants->swerve.driveEnableCurrentLimit,
		constants->swerve.driveContinuousCurrentLimit,
		constants->swerve.drivePeakCurrentLimit,
		constants->swerve.drivePeakCurrentDuration
	};

	swerveDriveConfig.slot0.kP = constants->swerve.driveKP;
	swerveDriveConfig.slot0.kI = constants->swerve.driveKI;
	swerveDriveConfig.slot0.kD = constants->swerve.driveKD;
	swerveDriveConfig.slot0.kF = constants->swerve.driveKF;
	swerveDriveConfig.supplyCurrLimit = driveSupplyLimit;
	swerveDriveConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
	swerveDriveConfig.openloopRamp = constants->swerve.openLoopRamp;
	swerveDriveConfig.closedloopRamp = constants->swerve.closedLoopRamp;

	// Swerve Encoder Configuration
	swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
	swerveCanCoderConfig.sensorDirection = constants->swerve.canCoderInvert;
	swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
	swerveCanCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
}