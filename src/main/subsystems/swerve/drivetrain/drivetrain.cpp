// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivetrain.h"

drivetrain::drivetrain() = default;

drivetrain::drivetrain(Constants * constants) {
    this->constants = constants;

    zeroGyro();

    swerveOdometry = new frc::SwerveDriveOdometry(constants->swerve.swerveKinematics, getYaw());
    swerveModules[0] = new SwerveModule(constants, 0, constants->swerve.Mod0);
    swerveModules[1] = new SwerveModule(constants, 1, constants->swerve.Mod1);
    swerveModules[2] = new SwerveModule(constants, 2, constants->swerve.Mod2);
    swerveModules[3] = new SwerveModule(constants, 3, constants->swerve.Mod3);
}

void drivetrain::drive(frc::Translation2d translation, units::radians_per_second_t rotation, bool fieldRelative, bool isOpenLoop) {
    wpi::array<frc::SwerveModuleState, 4> swerveModuleStates{ constants->swerve.swerveKinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            translation.X(),
            translation.Y(),
            rotation,
            getYaw())
        : frc::ChassisSpeeds {
            translation.X(),
            translation.Y(),
            rotation}) };
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&swerveModuleStates, constants->swerve.maxSpeed);

    for (int i = 0; i < 4; i++) {
        swerveModules[i]->setDesiredState(swerveModuleStates[swerveModules[i]->getModuleNumber()], isOpenLoop);
    }
}

void drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, 4>  desiredStates) {
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&desiredStates, constants->swerve.maxSpeed);

    for (int i = 0; i < 4; i++) {
        swerveModules[i]->setDesiredState(desiredStates[swerveModules[i]->getModuleNumber()], false);
    }
}

frc::Pose2d drivetrain::getPose() {
    return swerveOdometry->GetPose();
}

void drivetrain::resetOdomentry(frc::Pose2d pose) {
    swerveOdometry->ResetPosition(pose, getYaw());
}

wpi::array<frc::SwerveModuleState, 4> drivetrain::getStates() {

    wpi::array<frc::SwerveModuleState*, 4> states{ swerveModules[0]->getState(), swerveModules[1]->getState(), swerveModules[2]->getState(), swerveModules[3]->getState() };
}

void drivetrain::zeroGyro() {
    navx.ZeroYaw();
}

frc::Rotation2d drivetrain::getYaw() {
    if (navx.IsMagnetometerCalibrated()) {
        frc::Rotation2d yaw(units::degree_t(navx.GetFusedHeading()));
        return yaw;
    }

    frc::Rotation2d yaw(units::degree_t(360.0 - navx.GetYaw()));
    return yaw;
}

// This method will be called once per scheduler run
void drivetrain::Periodic() {
    swerveOdometry->Update(getYaw(), getStates);

    for (int i = 0; i < 4; i++) {
        frc::SmartDashboard::PutNumber("Mod " + swerveModules[i]->getModuleNumber() + ' Cancoder', units::unit_cast<double>(swerveModules[i]->getCanCoder().Degrees()));
        frc::SmartDashboard::PutNumber("Mod " + swerveModules[i]->getModuleNumber() + ' Integrated', units::unit_cast<double>(swerveModules[i]->getState()->angle.Degrees()));
        frc::SmartDashboard::PutNumber("Mod " + swerveModules[i]->getModuleNumber() + ' Velocity', units::unit_cast<double>(swerveModules[i]->getState()->speed));
        frc::SmartDashboard::PutNumber("Mod " + swerveModules[i]->getModuleNumber() + ' Target Angle', units::unit_cast<double>(swerveModules[i]->getLastDesiredState().angle.Degrees()));
    }
