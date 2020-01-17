/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoDrive.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>

frc::Trajectory generateTrajectory() {
  frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson("/home/lvuser/deploy/Default.wpilib.json");
  return trajectory;
}

AutoDrive::AutoDrive() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoDrive::Execute() {
bool AutoDrive::IsFinished() { return false; }
