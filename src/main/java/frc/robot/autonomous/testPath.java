/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import frc.robot.commands.PurePursuitCommand;
import frc.robot.navigation.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class testPath extends CommandGroup {
  boolean isPurePursuit;

  public testPath(boolean type) {
    if (type) {
      Point[] path = { new Point(1, 1, 0), new Point(157.25, 1) };
      addSequential(new PurePursuitCommand(path));
    }
  }
}
