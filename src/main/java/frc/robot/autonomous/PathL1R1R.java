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

public class PathL1R1R extends CommandGroup {
  boolean isPurePursuit;
  public PathL1R1R(boolean type) {
    isPurePursuit = type;
    if (isPurePursuit) {
      Point[] path = {new Point(1,1,0), new Point(1, 49.28), new Point(94.15, 142.35)};
      addSequential(new PurePursuitCommand(path));
    } else {
      addSequential( new DriveToTargetCommand(48.28));
      addSequential( new RotateToTargetCommand(45));
      addSequential( new DriveToTargetCommand(131.63));
    }
  }
}
