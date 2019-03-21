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
  private double lastLeftEncoderTicks = 0;
  private double lastRightEncoderTicks = 0;

  public PurePursuitCommand(Point[] path) {
    purePursuitPath = path;
    requires(Robot.m_drivesubsystem);
    purePursuit = new Controller(path, 0.8, 0.001);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drivesubsystem.ahrs.reset();
    lastLeftEncoderTicks = Robot.m_drivesubsystem.frontLeftTalon.getSelectedSensorPosition();
    lastRightEncoderTicks = Robot.m_drivesubsystem.frontRightTalon.getSelectedSensorPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double newLeftEncoderTicks = Robot.m_drivesubsystem.frontLeftTalon.getSelectedSensorPosition();
    double newRightEncoderTicks = Robot.m_drivesubsystem.frontRightTalon.getSelectedSensorPosition();
    
    double leftChange = Robot.m_drivesubsystem.talonUnitsToInches(newLeftEncoderTicks - lastLeftEncoderTicks);
    double rightChange = Robot.m_drivesubsystem.talonUnitsToInches(newRightEncoderTicks - lastRightEncoderTicks);

    isFinished = purePursuit.controlLoop(leftChange, rightChange, Robot.m_drivesubsystem.ahrs.getYaw());
    
    lastLeftEncoderTicks = newLeftEncoderTicks;
    lastRightEncoderTicks = newRightEncoderTicks;
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
    isFinished = false;
  }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
