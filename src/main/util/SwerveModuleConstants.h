#pragma once

#include <units/angle.h>

struct SwerveModuleConstants {
    int driveMotorID;
    int angleMotorID;
    int canCoderID;
    units::degree_t angleOffset;
};
