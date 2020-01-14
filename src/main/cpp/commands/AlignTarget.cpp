/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AlignTarget.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "subsystems/Drivetrain.h"

AlignTarget::AlignTarget(Drivetrain *drive) :
  drivetrain(drive)
{
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  AddRequirements({ drive });
}

// Called just before this Command runs the first time
void AlignTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AlignTarget::Execute() {
  bool hasTarget = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0);

  if (hasTarget) {
    double x = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0);

    drivetrain->SetAllSpeed(x / 30.0, -x / 30.0, x / 30.0, -x / 30.0);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool AlignTarget::IsFinished() { return false; }

// Called once after isFinished returns true
void AlignTarget::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AlignTarget::Interrupted() {}
