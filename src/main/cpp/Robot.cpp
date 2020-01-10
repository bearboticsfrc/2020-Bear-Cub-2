/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "ctre/Phoenix.h"

TalonSRX srx3 = {3};
TalonSRX srx4 = {4};

TalonSRX srx8 = {8};
TalonSRX srx6 = {6};

void Robot::RobotInit() {
    
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
    srx3.Set(ControlMode::PercentOutput, 0.2);
    srx4.Set(ControlMode::PercentOutput, 0.2);
    srx8.Set(ControlMode::PercentOutput, 0.2);
    srx6.Set(ControlMode::PercentOutput, 0.2);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
