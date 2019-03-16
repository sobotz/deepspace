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

import java.util.HashMap;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;

public class PurePursuitCommand extends Command {
  Point[] purePursuitPath;
  Controller purePursuit;
  private HashMap<String, Double> wV = new HashMap<>();
  private Timer time;

  public PurePursuitCommand(Point[] path) {
    purePursuitPath = path;
    requires(Robot.m_drivesubsystem);
    purePursuit = new Controller(path, 0.8, 0.001);
    time = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //this.wV = purePursuit.controlLoop(Robot.m_drivesubsystem.talonUnitsToInches(Robot.m_drivesubsystem.frontLeftTalon), Robot.m_drivesubsystem.talonUnitsToInches(Robot.m_drivesubsystem.frontRightTalon), Robot.m_drivesubsystem.ahrs.getYaw(),Robot.m_drivesubsystem.talonVelocityUnitsToNormal(Robot.m_drivesubsystem.frontLeftTalon),Robot.m_drivesubsystem.talonVelocityUnitsToNormal(Robot.m_drivesubsystem.frontRightTalon), time.get());
    System.out.print(wV.get("Left") + " " + wV.get("Right"));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
