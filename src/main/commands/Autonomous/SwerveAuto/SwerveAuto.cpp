// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SwerveAuto::SwerveAuto(Drivetrain* swerve) : drivetrain{ swerve } {
	// Add your commands here, e.g.
	// AddCommands(FooCommand(), BarCommand());

	frc::TrajectoryConfig config(drivetrain->constants.autoConstants.kMaxSpeed, drivetrain->constants.autoConstants.kMaxAcceleration);
	config.SetKinematics(drivetrain->constants.swerve.swerveKinematics);

	frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
		frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
		{ frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m) },
		frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
		config
	);

	frc::ProfiledPIDController<units::radian> thetaController(
		drivetrain->constants.autoConstants.kPThetaController,
		0, 0,
		drivetrain->constants.autoConstants.kThetaControllerConstraints
	);
	thetaController.EnableContinuousInput(-180_deg, 180_deg);

	frc2::SwerveControllerCommand<4> swerveControllerCommand(
		trajectory,
		[this]() { return drivetrain->getPose(); },
		drivetrain->constants.swerve.swerveKinematics,
		frc2::PIDController(drivetrain->constants.autoConstants.kPXController, 0, 0),
		frc2::PIDController(drivetrain->constants.autoConstants.kPYController, 0, 0),
		thetaController,
		[this](auto moduleStates) {drivetrain->setModuleStates(moduleStates);},
		{ drivetrain }
	);

	AddCommands(frc2::InstantCommand([&] {drivetrain->resetOdometry(trajectory.InitialPose());}, { drivetrain }),
		swerveControllerCommand);
}
