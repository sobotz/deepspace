/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command. You can replace me with your own command.
 */
public class ArticulationCommand extends Command {
  private int position;
  private String ControlMode;

  public ArticulationCommand() {
    // Use requires() here to declare subsystem dependencies
    ControlMode = "manual";
    requires(Robot.m_intake);
  }

  public ArticulationCommand(int p) {
    // Use requires() here to declare subsystem dependencies
    position = p;
    ControlMode = "controlLoop";
    requires(Robot.m_intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (ControlMode == "manual") {
      Robot.m_intake.control(Robot.m_oi.operatorJoystick.getRawAxis(1), Robot.m_oi.operatorJoystick.getRawAxis(5),
          Robot.m_oi.operatorJoystick.getRawAxis(3), Robot.m_oi.operatorJoystick.getRawAxis(2));
    } else {
      Robot.m_intake.articulateArms(position);
    }
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
