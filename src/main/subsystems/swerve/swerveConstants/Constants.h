#include "./util/SwerveModuleConstants.h"
#include "ModuleIDs.h"

#include <units/units.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <wpi/math>
#include <ctre/Phoenix.h>

struct Swerve {
	// Always ensure Gyro is CCW+ CW-
	bool invertGyro = false;

	// Drivetrain Constants
	double trackWidth = units::meter_t(20.75_in).to<double>();
	double wheelBase = units::meter_t(20.75_in).to<double>();
	double wheelDiameter = units::meter_t(4_in).to<double>();
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
	int angleContinuousCurrentLimit = 25;
	int anglePeakCurrentLimit = 40;
	double anglePeakCurrentDuration = 0.1;
	bool angleEnableCurrentLimit = true;

	// Swerve Drive Current Limit
	int driveContinuousCurrentLimit = 35;
	int drivePeakCurrentLimit = 60;
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
	units::unit_t<units::velocity::meters_per_second, double, units::linear_scale> driveKV = (10_mps);
	units::unit_t<units::acceleration::meters_per_second_squared, double, units::linear_scale> driveKA = (20_mps_sq);

	// Swerve Profiling Values
	units::meters_per_second_t maxSpeed = 4.5_mps;
	units::meters_per_second_t maxAngularVelocity = 11.5_mps;

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
	SwerveModuleConstants Mod0{ 2,1,9,42.62 };

	// Front Right Module - Module 1
	SwerveModuleConstants Mod1{ 4,3,10,114.78 };

	// Back Left Module - Module 2
	SwerveModuleConstants Mod2{ 8,7,12,18.54 };

	// Back Right Module - Module 3
	SwerveModuleConstants Mod3{ 6,5,11,29.26 };
};

struct AutoConstants {
	units::meters_per_second_t kMaxSpeed = 3_mps;
	units::meters_per_second_squared_t kMaxAcceleration = 3_mps_sq;
	units::radian_t kMaxAngularSpeedRadiansPerSecond = 180_deg;
	units::radian_t kMaxAngularAccelerationRadiansPerSecondSquared = 180_deg;

	double kPXController = 1;
	double kPYController = 1;
	double kPThetaController = 1;

	// Constraint for the motion profilied robot angle controller
	frc::TrapezoidProfile<units::radian_t>::Constraints kThetaControllerConstraints{ kMaxAngularSpeedRadiansPerSecond , kMaxAngularAccelerationRadiansPerSecondSquared };
};

struct Constants {
	double stickDeadBand = 0.1;

	Swerve swerve;
	AutoConstants autoConstants;

};
