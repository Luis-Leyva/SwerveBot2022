#pragma once

class Conversions {
public:
    /**
    * @param counts Falcon Counts
    * @param gearRatio Gear Ratio between Falcon and Mechanism
    * @return Degrees of Rotation of Mechanism
    */
    double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
    * @param degrees Degrees of rotation of Mechanism
    * @param gearRatio Gear Ratio between Falcon and Mechanism
    * @return Falcon Counts
    */
    double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
    * @param velocityCounts Falcon Velocity Counts
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return RPM of Mechanism
    */
    double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
    * @param RPM RPM of mechanism
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return RPM of Mechanism
    */
    double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
    * @param velocitycounts Falcon Velocity Counts
    * @param circumference Circumference of Wheel
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return Falcon Velocity Counts
    */
    double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
    * @param velocity Velocity MPS
    * @param circumference Circumference of Wheel
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return Falcon Velocity Counts
    */
    double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
};