/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <vector>
#include <cmath>

using frc::Translation2d;
using frc::Transform2d;
using frc::Rotation2d;

// TODO: Is it 512, or actually 520?
const double EDGES_PER_TURN = 512.0;
const double WHEEL_DIAMETER = 0.103;
const double WHEEL_CIRCUM = 0.103 * 3.1415926;
const double EDGES_PER_METER = 512 / WHEEL_CIRCUM;

Drivetrain::Drivetrain() :
    backLeft(DriveConsts::BACK_LEFT_ID),
    backRight(DriveConsts::BACK_RIGHT_ID),
    frontLeft(DriveConsts::FRONT_LEFT_ID),
    frontRight(DriveConsts::FRONT_RIGHT_ID)
{
    backLeft.SetInverted(true);
    frontLeft.SetInverted(true);

    backLeft.SetSensorPhase(true);
    backRight.SetSensorPhase(true);
    frontLeft.SetSensorPhase(true);
    frontRight.SetSensorPhase(true);

    backLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    backRight.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    frontLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);
    frontRight.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0);

    backLeft.Config_kF(0, 1023.0 / 295.0);
    backRight.Config_kF(0, 1023.0 / 215.0);
    frontLeft.Config_kF(0, 1023.0 / 312.0);
    frontRight.Config_kF(0, 1023.0 / 185.0);

    backLeft.Config_kP(0, 8.0);
    backRight.Config_kP(0, 8.0);
    frontLeft.Config_kP(0, 8.0);
    frontRight.Config_kP(0, 8.0);

    backLeft.Config_kI(0, 0.0);
    backRight.Config_kI(0, 0.0);
    frontLeft.Config_kI(0, 0.0);
    frontRight.Config_kI(0, 0.0);

    backLeft.Config_kD(0, 0.0);
    backRight.Config_kD(0, 0.0);
    frontLeft.Config_kD(0, 0.0);
    frontRight.Config_kD(0, 0.0);

    backLeft.SetSelectedSensorPosition(0);
    backRight.SetSelectedSensorPosition(0);
    frontLeft.SetSelectedSensorPosition(0);
    frontRight.SetSelectedSensorPosition(0);
}

units::meter_t from_talon_distance(double dist) {
    return units::meter_t(dist * EDGES_PER_METER);
}

units::meters_per_second_t from_talon_velocity(double speed) {
    return units::meters_per_second_t(speed * EDGES_PER_METER / 10.0);
}

class RollingAverage {
public:
    void add_sample(double sample) {
        history[idx] = sample;
        idx += 1;
        idx %= history.size();
    }

    double get_total() {
        double total = 0.0;

        for (std::size_t i = 0; i < history.size(); ++i) {
            total += history[i] / history.size();
        }

        return total;
    }

private:
    std::vector<double> history = std::vector<double>(10);
    std::size_t idx = 0;
};

// This method will be called once per scheduler run
void Drivetrain::Periodic() {
    TalonSRX *motors[] = { &backLeft, &backRight, &frontLeft, &frontRight };

    static RollingAverage averages[4] = { RollingAverage(), RollingAverage(), RollingAverage(), RollingAverage() };

    double changes[4];

    for (int i = 0; i < 4; ++i) {
        averages[i].add_sample(motors[i]->GetSelectedSensorVelocity());
        changes[i] = prev[i];
        prev[i] = motors[i]->GetSelectedSensorPosition();
        changes[i] = prev[i] - changes[i];
        frc::SmartDashboard::PutNumber("Motor " + std::to_string(i), averages[i].get_total());
    }

    double leftSpeed = (changes[0] + changes[2]) / 2.0;
    double rightSpeed = (changes[1] + changes[3]) / 2.0;

    UpdatePose(from_talon_distance(leftSpeed), from_talon_distance(rightSpeed));
}

/*
|x'|   |cos(w dt) -sin(w dt) 0|   | x - ICC_x |     |ICC_x|
|y'| = |sin(w dt)  cos(w dt) 0| * | y - ICC_y |  +  |ICC_y|
|t'|   |    0          0     1|   |     t     |     | w dt|

R = l / 2 * (V_r + V_l) / (V_r - V_l)
ICC = [x - R sin(t), y + R cos(t)]
*/

const units::meter_t WHEELBASE = units::inch_t(12.0);

void Drivetrain::UpdatePose(units::meter_t leftDist, units::meter_t rightDist) {
    if (std::abs((leftDist - rightDist).to<double>()) < 0.0001) {
        pose += Transform2d(
            Translation2d(
                leftDist,
                rightDist
            ),
            Rotation2d()
        );
    } else {
        units::meter_t r = WHEELBASE / 2.0 * (leftDist + rightDist) / (leftDist - rightDist);
        units::meter_t icc_x = pose.Translation().X() - r * pose.Rotation().Sin();
        units::meter_t icc_y = pose.Translation().Y() + r * pose.Rotation().Cos();
        units::radian_t w = units::radian_t(((rightDist - leftDist) / WHEELBASE).to<double>());

        pose += Transform2d{ Translation2d(-icc_x, -icc_y), Rotation2d() };
        pose.Translation().RotateBy(Rotation2d{ w });
        pose += Transform2d{ Translation2d(icc_x, icc_y), Rotation2d(w) };
    }
}

void Drivetrain::SetSpeed(double speed) {
    SetAllSpeed(speed, speed, speed, speed);
}

double to_talon_velocity(units::meters_per_second_t speed) {
    // Speed is in Edges per 100ms
    return speed.to<double>() * EDGES_PER_METER / 10.0;
}

void Drivetrain::SetSpeeds(double leftSpeed, double rightSpeed) {
    SetAllSpeed(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
}

void Drivetrain::SetSpeeds(units::meters_per_second_t lSpeed, units::meters_per_second_t rSpeed) {
    double leftSpeed = to_talon_velocity(lSpeed);
    double rightSpeed = to_talon_velocity(rSpeed);

    backLeft.Set(ControlMode::Velocity, leftSpeed);
    frontLeft.Set(ControlMode::Velocity, leftSpeed);

    backRight.Set(ControlMode::Velocity, rightSpeed);
    frontRight.Set(ControlMode::Velocity, rightSpeed);
}

frc::Pose2d Drivetrain::GetPose() {
    return pose;
}

void Drivetrain::SetAllSpeed(double backLeftSpeed, double backRightSpeed, double frontLeftSpeed, double frontRightSpeed) {
    backLeftSpeed = std::min(1.0, std::max(-1.0, backLeftSpeed));
    backRightSpeed = std::min(1.0, std::max(-1.0, backRightSpeed));
    frontLeftSpeed = std::min(1.0, std::max(-1.0, frontLeftSpeed));
    frontRightSpeed = std::min(1.0, std::max(-1.0, frontRightSpeed));

    backLeftSpeed *= 150.0;
    backRightSpeed *= 150.0;
    frontLeftSpeed *= 150.0;
    frontRightSpeed *= 150.0;

    /*backLeft.Set(ControlMode::PercentOutput, backLeftSpeed);
    backRight.Set(ControlMode::PercentOutput, backRightSpeed);
    frontLeft.Set(ControlMode::PercentOutput, frontLeftSpeed);
    frontRight.Set(ControlMode::PercentOutput, frontRightSpeed);*/

    backLeft.Set(ControlMode::Velocity, backLeftSpeed);
    backRight.Set(ControlMode::Velocity, backRightSpeed);
    frontLeft.Set(ControlMode::Velocity, frontLeftSpeed);
    frontRight.Set(ControlMode::Velocity, frontRightSpeed);
}