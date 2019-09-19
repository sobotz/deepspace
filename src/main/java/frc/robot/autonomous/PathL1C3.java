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

public class PathL1C3 extends CommandGroup {
  boolean isPurePursuit;

  public PathL1C3(boolean type) {
    if (isPurePursuit) {
      Point[] path = { new Point(1, 1, 0), new Point(1, 181.8), new Point(-4, 186.8), new Point(-4, 191.8),
          new Point(1, 196.8), new Point(3.13, 196.8) };
      addSequential(new PurePursuitCommand(path));
    } else {
      addSequential(new DriveToTargetCommand(180.8));
      addSequential(new RotateToTargetCommand(-45));
      addSequential(new DriveToTargetCommand(7.07));
      addSequential(new RotateToTargetCommand(45));
      addSequential(new DriveToTargetCommand(5));
      addSequential(new RotateToTargetCommand(45));
      addSequential(new DriveToTargetCommand(7.07));
      addSequential(new RotateToTargetCommand(45));
      addSequential(new DriveToTargetCommand(2.13));
    }
  }
}
