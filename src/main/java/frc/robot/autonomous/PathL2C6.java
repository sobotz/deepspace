/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import frc.robot.commands.DriveToTargetCommand;
import frc.robot.commands.PurePursuitCommand;
import frc.robot.commands.RotateToTargetCommand;
import frc.robot.navigation.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathL2C6 extends CommandGroup {
  /**
   * Add your docs here.
   */
  boolean isPurePursuit;

  public PathL2C6(boolean type) {
    isPurePursuit = type;
    if (isPurePursuit) {
      Point[] path = { new Point(1, 1, 0), new Point(1, 51), new Point(61, 111), new Point(61, 195.05),
          new Point(56, 200.05), new Point(29.87, 200.05) };
      addSequential(new PurePursuitCommand(path));
    } else {
      addSequential(new DriveToTargetCommand(50));
      addSequential(new RotateToTargetCommand(45));
      addSequential(new DriveToTargetCommand(84.85));
      addSequential(new RotateToTargetCommand(-45));
      addSequential(new DriveToTargetCommand(84.85));
      addSequential(new RotateToTargetCommand(-45));
      addSequential(new DriveToTargetCommand(7.07));
      addSequential(new RotateToTargetCommand(-45));
      addSequential(new DriveToTargetCommand(26.13));
    }
  }
}
