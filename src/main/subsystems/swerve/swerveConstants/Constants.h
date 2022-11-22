#pragma once

#include "./util/SwerveModuleConstants.h"

#include <units/length.h>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/base.h>
#include <units/angle.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <wpi/numbers>
#include <ctre/Phoenix.h>

struct Swerve {
	// Always ensure Gyro is CCW+ CW-
	bool invertGyro = false;

	// Drivetrain Constants
	double trackWidth = 0.53;
	double wheelBase = 0.53;
	double wheelDiameter = 0.1016;
	double wheelCircumference = wheelDiameter * M_PI;

	double openLoopRamp = 0.25;
	double closedLoopRamp = 0.0;

	double driveGearRatio = (6.75 / 1.0); // 6.75:1
	double angleGearRatio = ((150.0 / 7.0) / 1.0); // 150:7:1

	// Swerve Drive Kinematics
	frc::SwerveDriveKinematics<4> swerveKinematics{
		frc::Translation2d{units::meter_t(trackWidth / 2.0), units::meter_t(wheelBase / 2.0)},
		frc::Translation2d{units::meter_t(trackWidth / 2.0), units::meter_t(-wheelBase / 2.0)},
		frc::Translation2d{units::meter_t(-trackWidth / 2.0), units::meter_t(wheelBase / 2.0)},
		frc::Translation2d{units::meter_t(-trackWidth / 2.0), units::meter_t(-wheelBase / 2.0)}
	};

	// Swerve Angle Current Limit
	double angleContinuousCurrentLimit = 25.0;
	double anglePeakCurrentLimit = 40.0;
	double anglePeakCurrentDuration = 0.1;
	bool angleEnableCurrentLimit = true;

	// Swerve Drive Current Limit
	double driveContinuousCurrentLimit = 35.0;
	double drivePeakCurrentLimit = 60.0;
	double drivePeakCurrentDuration = 0.1;
	bool driveEnableCurrentLimit = true;

	// Angle Motor PID
	double angleKP = 0.1;
	double angleKI = 0.0;
	double angleKD = 0.0;
	double angleKF = 0.0;

	// Drive Motor PID
	double driveKP = 0.01;
	double driveKI = 0.0;
	double driveKD = 0.0;
	double driveKF = 0.0;

	// Drive Motor Characterization
	units::volt_t driveKS = (12.0_V); // divide by 12.0 to convert volts to percent output for CTRE
	units::unit_t<frc::SimpleMotorFeedforward<units::meter>::kv_unit> driveKV{ 10.00 };
	units::unit_t<frc::SimpleMotorFeedforward<units::meter>::ka_unit> driveKA{ 20.00 };

	// Swerve Profiling Values
	units::meters_per_second_t maxSpeed = 4.5_mps;
	units::radians_per_second_t maxAngularVelocity = 11.5_rad_per_s;

	// Neutral Modes
	NeutralMode angleNeutralMode = NeutralMode::Coast;
	NeutralMode driveNeutralMode = NeutralMode::Brake;

	// Motor Inverts 
	bool angleMotorInvert = true;
	bool driveMotorInvert = false;

	// Angle Encoder Inverts
	bool canCoderInvert = true;

	/* Module Specific Constants */
	// Front Left Module - Module 0
	SwerveModuleConstants Mod0{ 2,1,9,42.62_deg };

	// Front Right Module - Module 1
	SwerveModuleConstants Mod1{ 4,3,10,114.78_deg };

	// Back Left Module - Module 2
	SwerveModuleConstants Mod2{ 8,7,12,18.54_deg };

	// Back Right Module - Module 3
	SwerveModuleConstants Mod3{ 6,5,11,29.26_deg };

	TalonFXConfiguration swerveAngleConfig;
	TalonFXConfiguration swerveDriveConfig;
	CANCoderConfiguration swerveCanCoderConfig;

	// Swerve Angle Motor Configuration
	SupplyCurrentLimitConfiguration angleSupplyLimit{
		angleEnableCurrentLimit,
		angleContinuousCurrentLimit,
		anglePeakCurrentLimit,
		anglePeakCurrentDuration
	};

	// Swerve Drive Motor Configuration
	SupplyCurrentLimitConfiguration driveSupplyLimit{
		driveEnableCurrentLimit,
		driveContinuousCurrentLimit,
		drivePeakCurrentLimit,
		drivePeakCurrentDuration
	};

	void configure() {
		swerveAngleConfig.slot0.kP = angleKP;
		swerveAngleConfig.slot0.kI = angleKI;
		swerveAngleConfig.slot0.kD = angleKD;
		swerveAngleConfig.slot0.kF = angleKF;
		swerveAngleConfig.supplyCurrLimit = angleSupplyLimit;
		swerveAngleConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;

		swerveDriveConfig.slot0.kP = driveKP;
		swerveDriveConfig.slot0.kI = driveKI;
		swerveDriveConfig.slot0.kD = driveKD;
		swerveDriveConfig.slot0.kF = driveKF;
		swerveDriveConfig.supplyCurrLimit = driveSupplyLimit;
		swerveDriveConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
		swerveDriveConfig.openloopRamp = openLoopRamp;
		swerveDriveConfig.closedloopRamp = closedLoopRamp;

		// Swerve Encoder Configuration
		swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
		swerveCanCoderConfig.sensorDirection = canCoderInvert;
		swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
		swerveCanCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
	}
};

struct AutoConstants {
	units::meters_per_second_t kMaxSpeed = 3_mps;
	units::meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;
	units::radians_per_second_t kMaxAngularSpeed = wpi::numbers::pi * 1_rad_per_s;
	units::radians_per_second_squared_t kMaxAngularAcceleration = wpi::numbers::pi * 1_rad_per_s_sq;

	double kPXController = 1;
	double kPYController = 1;
	double kPThetaController = 1;

	// Constraint for the motion profilied robot angle controller
	frc::TrapezoidProfile<units::radians>::Constraints kThetaControllerConstraints{ kMaxAngularSpeed , kMaxAngularAcceleration };
};


struct Constants {
	double stickDeadBand = 0.1;

	Swerve swerve;
	AutoConstants autoConstants;

};
