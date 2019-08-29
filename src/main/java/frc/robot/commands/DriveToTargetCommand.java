/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveToTargetCommand extends Command {
  double distance;
  boolean isFinished = false;

  public DriveToTargetCommand(double myDistance) {
    requires(Robot.m_drivesubsystem);
    distance = myDistance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Do nothing
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    isFinished = Robot.m_drivesubsystem.driveToTarget(distance, 2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drivesubsystem.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_drivesubsystem.reset();
  }
}
