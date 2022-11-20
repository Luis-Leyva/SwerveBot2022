#pragma once

struct Conversions {
public:
    /**
    * @param counts Falcon Counts
    * @param gearRatio Gear Ratio between Falcon and Mechanism
    * @return Degrees of Rotation of Mechanism
    */
    static units::degree_t falconToDegrees(double counts, double gearRatio) {
        units::degree_t degrees{ counts * (360.0 / (gearRatio * 2048.0)) };
        return degrees;
    }

    /**
    * @param degrees Degrees of rotation of Mechanism
    * @param gearRatio Gear Ratio between Falcon and Mechanism
    * @return Falcon Counts
    */
    static double degreesToFalcon(units::degree_t degrees, double gearRatio) {
        double ticks = units::unit_cast<double>(degrees) / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
    * @param velocityCounts Falcon Velocity Counts
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return RPM of Mechanism
    */
    static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
    * @param RPM RPM of mechanism
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return RPM of Mechanism
    */
    static double RPMToFalcon(double RPM, double gearRatio) {
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
    static units::meters_per_second_t falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        units::meters_per_second_t wheelMPS{ (wheelRPM * circumference) / 60.0 };
        return wheelMPS;
    }

    /**
    * @param velocity Velocity MPS
    * @param circumference Circumference of Wheel
    * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
    * @return Falcon Velocity Counts
    */
    static double MPSToFalcon(units::velocity::meters_per_second_t velocity, double circumference, double gearRatio) {
        double wheelRPM = ((units::unit_cast<double>(velocity) * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
};