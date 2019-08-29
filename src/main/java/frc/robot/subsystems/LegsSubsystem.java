/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class LegsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  DoubleSolenoid frontLeftLegs = new DoubleSolenoid(4, 5);
  DoubleSolenoid backLeg = new DoubleSolenoid(6, 7);

  public LegsSubsystem() {

  }

  public void shift(int action) {
    switch (action) {
    case 8:
      frontLeftLegs.set(DoubleSolenoid.Value.kReverse);
      break;
    case 7:
      frontLeftLegs.set(DoubleSolenoid.Value.kForward);
      break;
    case 9:
      backLeg.set(DoubleSolenoid.Value.kForward);
      break;
    case 10:
      backLeg.set(DoubleSolenoid.Value.kReverse);
      break;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}