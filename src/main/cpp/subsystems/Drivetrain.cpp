/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>

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

    backLeft.Config_kF(0, 1023.0 / 55.0);
    backRight.Config_kF(0, 1023.0 / 222.0);
    frontLeft.Config_kF(0, 1023.0 / 360.0);
    frontRight.Config_kF(0, 1023.0 / 127.0);

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

    for (int i = 0; i < 4; ++i) {
        averages[i].add_sample(motors[i]->GetSelectedSensorVelocity());
        frc::SmartDashboard::PutNumber("Motor " + std::to_string(i), averages[i].get_total());
    }
}

void Drivetrain::SetSpeed(double speed) {
    SetAllSpeed(speed, speed, speed, speed);
}


void Drivetrain::SetAllSpeed(double backLeftSpeed, double backRightSpeed, double frontLeftSpeed, double frontRightSpeed) {
    backLeftSpeed = std::min(1.0, std::max(-1.0, backLeftSpeed));
    backRightSpeed = std::min(1.0, std::max(-1.0, backRightSpeed));
    frontLeftSpeed = std::min(1.0, std::max(-1.0, frontLeftSpeed));
    frontRightSpeed = std::min(1.0, std::max(-1.0, frontRightSpeed));

    backLeftSpeed *= 50.0;
    backRightSpeed *= 50.0;
    frontLeftSpeed *= 50.0;
    frontRightSpeed *= 50.0;

    /*backLeft.Set(ControlMode::PercentOutput, backLeftSpeed);
    backRight.Set(ControlMode::PercentOutput, backRightSpeed);
    frontLeft.Set(ControlMode::PercentOutput, frontLeftSpeed);
    frontRight.Set(ControlMode::PercentOutput, frontRightSpeed);*/

    backLeft.Set(ControlMode::Velocity, backLeftSpeed);
    backRight.Set(ControlMode::Velocity, backRightSpeed);
    frontLeft.Set(ControlMode::Velocity, frontLeftSpeed);
    frontRight.Set(ControlMode::Velocity, frontRightSpeed);
}