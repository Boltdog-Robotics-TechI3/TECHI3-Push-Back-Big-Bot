#pragma once
#include "main.h"

inline bool skills = false;
inline bool match = true;
inline bool redAlliance = false;
inline bool blueAlliance = true;
inline int autoSelection = 0;

inline const double wheelDiameter = 2.75;
inline const double trackWidth = 12.5;
inline const double gearRatio = 1;

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

inline pros::MotorGroup frontLeftDrive({-16, 17});
inline pros::MotorGroup frontRightDrive({-9, 10});
inline pros::MotorGroup backLeftDrive({12, -14});
inline pros::MotorGroup backRightDrive({11, -13});


inline pros::Motor intake1(-18);
inline pros::Motor intake2(19);
inline pros::Motor midRoller(-1);
inline pros::Motor colorSorter(-20);
inline pros::Motor bottomRollers(-15);
inline pros::Motor loadOrLow(8);
inline pros::Motor lowOrHigh(6);
inline pros::Motor hopper(-7);
inline pros::adi::Pneumatics wing('A',true);

inline HolonomicDrivetrain drivetrain(&frontLeftDrive, &frontRightDrive, &backLeftDrive, &backRightDrive, wheelDiameter, trackWidth, gearRatio);

inline TrackingWheel verticalTrackingWheel(7, 2, 0, WheelPosition::LEFT);
inline TrackingWheel horizontalTrackingWheel(6, 2, 0, WheelPosition::BACK);

inline pros::IMU gyro(21);

inline Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &gyro);

inline HolonomicChassis chassis(&drivetrain, &odometry);