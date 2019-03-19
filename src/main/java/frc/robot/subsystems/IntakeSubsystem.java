/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ArticulationCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class IntakeSubsystem extends Subsystem {
  TalonSRX articulationTalon;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX rollerTalon;
  public TalonSRX rollerTalonSlave;
  TalonSRX dropTalon;


  public IntakeSubsystem() {
    articulationTalon = new TalonSRX(RobotMap.articulationMotor);
  //  rollerTalon = new TalonSRX(RobotMap.rollerMotor);
  //  dropTalon = new TalonSRX(RobotMap.dropMotor);

    articulationTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, 30);
  ///  dropTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, 30);

  articulationTalon.configFactoryDefault();

  articulationTalon.configPeakOutputReverse(-0.05);
  articulationTalon.configPeakOutputForward(0.05);

  }


  public double toDegree(){
    return (4096/360)*(articulationTalon.getSelectedSensorPosition()/4096);
  }

  public double degreeToTalonUnit(double degree){
    return degree*4096*360;
  } 

  public void control(double input){

    articulationTalon.set(ControlMode.PercentOutput, input);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   
    setDefaultCommand(new ArticulationCommand());
  }
}
