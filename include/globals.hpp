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

inline pros::MotorGroup frontLeftDrive({1, -16});
inline pros::MotorGroup frontRightDrive({-19, 20});
inline pros::MotorGroup backLeftDrive({-13, 14});
inline pros::MotorGroup backRightDrive({11, -12});

inline pros::MotorGroup intake({2, -3});
inline pros::Motor midRoller(-1);
inline pros::Motor colorSorter(-20);
inline pros::Motor bottomRollers(-15);
inline pros::Motor loadOrMid(-8);
inline pros::Motor midOrHigh(6);
inline pros::Motor hopper(-7);
inline pros::adi::Pneumatics wing('A',true);

inline HolonomicDrivetrain drivetrain(&frontLeftDrive, &frontRightDrive, &backLeftDrive, &backRightDrive, wheelDiameter, trackWidth, gearRatio);

inline TrackingWheel verticalTrackingWheel(17, 2.08, 0, WheelPosition::LEFT);
inline TrackingWheel horizontalTrackingWheel(18, 2.08, 0, WheelPosition::BACK);

inline pros::IMU gyro(5);

inline Odometry odometry(&verticalTrackingWheel, NULL, &horizontalTrackingWheel, &gyro);

inline PIDController lateral(.2, 0, 0);
inline PIDController turn(60, 0.2, 3);
inline PIDController align(10, 0, 0);

inline HolonomicChassis chassis(&drivetrain, &odometry, &lateral, &turn, &align);