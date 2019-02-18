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
      Point[] path = {new Point(1,1,0), new Point(129.54,1), new Point(129.54,-86.625), new Point(142.27, -99.355)};
      addSequential(new PurePursuitCommand(path));
    } else {
      addSequential( new DriveToTargetCommand(128.54));
      addSequential( new RotateToTargetCommand(90));
      addSequential( new DriveToTargetCommand(87.265));
      addSequential( new RotateToTargetCommand(-45));
      addSequential( new DriveToTargetCommand(18));
    }
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
