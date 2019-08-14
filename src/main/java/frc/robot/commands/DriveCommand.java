/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveCommand extends Command {
  public DriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_drivesubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drivesubsystem.m_drive.setSafetyEnabled(true);
    Robot.m_drivesubsystem.manualDrive(Robot.m_oi.driverJoystick.getRawAxis(1), Robot.m_oi.driverJoystick.getRawAxis(0));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_drivesubsystem.manualDrive(Robot.m_oi.driverJoystick.getRawAxis(1), Robot.m_oi.driverJoystick.getRawAxis(0));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() { 
    Robot.m_drivesubsystem.m_drive.stopMotor();
    Robot.m_drivesubsystem.m_drive.feedWatchdog();
    Robot.m_drivesubsystem.m_drive.feed();
    Robot.m_drivesubsystem.m_drive.setSafetyEnabled(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_drivesubsystem.m_drive.stopMotor();
    Robot.m_drivesubsystem.m_drive.feedWatchdog();
    Robot.m_drivesubsystem.m_drive.feed();
    Robot.m_drivesubsystem.m_drive.setSafetyEnabled(false);
  }
}
