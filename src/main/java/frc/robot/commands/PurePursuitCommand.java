/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.*;
import frc.robot.navigation.Controller;
import frc.robot.navigation.Point;

import edu.wpi.first.wpilibj.command.Command;

public class PurePursuitCommand extends Command {
  Point[] purePursuitPath;
  Controller purePursuit;
  private boolean isFinished = false;

  public PurePursuitCommand(Point[] path) {
    purePursuitPath = path;
    requires(Robot.m_drivesubsystem);
    purePursuit = new Controller(path, 0.8, 0.001);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Do nothing
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    isFinished = purePursuit.controlLoop(
        Robot.m_drivesubsystem.talonUnitsToInches(Robot.m_drivesubsystem.frontLeftTalon),
        Robot.m_drivesubsystem.talonUnitsToInches(Robot.m_drivesubsystem.frontRightTalon),
        Robot.m_drivesubsystem.ahrs.getYaw());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Do nothing
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // Do nothing
  }
}
