/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

#include "commands/ExampleCommand.h"
#include "commands/DefaultDrive.h"
#include "commands/AlignTarget.h"
#include "commands/AutoDrive.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  frc2::Command* GetTeleopCommand();

 private:
  // The robot's subsystems and commands are defined here...
  ExampleSubsystem m_subsystem;
  Drivetrain m_drivetrain;
  ExampleCommand m_autonomousCommand;
  DefaultDrive m_defaultDrive;
  AlignTarget m_alignTarget;
  std::unique_ptr<frc2::Command> m_ramsete;

  PigeonIMU pigeon;

  void ConfigureButtonBindings();
};
