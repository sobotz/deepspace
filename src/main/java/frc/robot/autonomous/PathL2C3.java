/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import frc.robot.Robot;
import frc.robot.commands.DriveToTargetCommand;
import frc.robot.commands.PurePursuitCommand;
import frc.robot.commands.RotateToTargetCommand;
import frc.robot.navigation.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathL2C3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  boolean isPurePursuit; 
  public PathL2C3(boolean type) {
    isPurePursuit = type;
    if (isPurePursuit) {
      Point[] path = {new Point(1,1,0), new Point(1,50), new Point(-59, 111), new Point(-59, 195.05), new Point(-54, 200.05), new Point(-27.87, 200.05)};
      addSequential(new PurePursuitCommand(path));
    } else {
      addSequential( new DriveToTargetCommand(50));
      addSequential( new RotateToTargetCommand(45));
      addSequential( new DriveToTargetCommand(84.85));
      addSequential( new RotateToTargetCommand(-45));
      addSequential( new DriveToTargetCommand(84.05));
      addSequential( new RotateToTargetCommand(-45));
      addSequential( new DriveToTargetCommand(7.07));
      addSequential( new RotateToTargetCommand(-45));
      addSequential( new DriveToTargetCommand(26.13));
    }
  }
}
