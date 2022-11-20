#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/units.h>
#include <Math.h>

struct CTREModuleState {
public:
	/**
	* @param scopeReference Current Angle
	* @param newAngle Target Angle
	* @return Closest angle within scope
	*/
	static double placeInAppropriate0To360Scope(units::degree_t scopeReferenceDegree, units::degree_t newAngleDegree) {
		double scopeReference = scopeReferenceDegree.to<double>();
		double newAngle = newAngleDegree.to<double>();
		double lowerBound;
		double upperBound;
		double lowerOffset = fmod(scopeReference, 360.00);
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		} else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		} else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}
	/**
	* Minimize the change in heading the desired swerve module state would require by potentially
	* reversing the direction the wheel spins. Customized from WPILib's version to include placing
	* in appropriate scope for CTRE onboard control.
	*
	* @param desiredState The desired state.
	* @param currentAngle The current module angle.
	*/
	static frc::SwerveModuleState Optimize(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.Degrees(), desiredState.angle.Degrees());
		double targetSpeed = desiredState.speed.to<double>();
		double delta = targetAngle - currentAngle.Degrees().to<double>();
		if (abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}

		units::meters_per_second_t targetSpeedUnit = units::meters_per_second_t(targetSpeed);
		units::degree_t targetAngleUnit = units::degree_t(targetAngle);
		frc::Rotation2d targetAngleRotation = frc::Rotation2d(targetAngleUnit);
		frc::SwerveModuleState newState{ targetSpeedUnit, targetAngleRotation };
		return newState;
	}
};