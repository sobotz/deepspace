/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ArticulationCommand;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;

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

  private ArrayList<Integer> velocities = new ArrayList<Integer>();
  private Timer recordPeakVelocityTimer = new Timer();
  private boolean recordPeakVelocityStarted = false;
  private int peakVelocity = 0;


  public IntakeSubsystem() {
    articulationTalon = new TalonSRX(RobotMap.articulationMotor);
    //dropTalon = new TalonSRX(RobotMap.dropMotor);

    rollerTalon = new TalonSRX(RobotMap.rollerMotor);
    rollerTalonSlave = new TalonSRX(RobotMap.rollerMotorSlave);


    dropTalon = new TalonSRX(RobotMap.dropMotor);

    articulationTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, 30);
  ///  dropTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, 30);

  articulationTalon.configFactoryDefault();
  rollerTalon.configFactoryDefault();
  rollerTalonSlave.configFactoryDefault();
  dropTalon.configFactoryDefault();

  articulationTalon.configPeakOutputReverse(-0.3);
  articulationTalon.configPeakOutputForward(0.3);

  rollerTalon.configPeakOutputReverse(-0.4);
  rollerTalon.configPeakOutputForward(0.4);

  dropTalon.configPeakOutputReverse(-0.3);
  dropTalon.configPeakOutputForward(0.3);




  articulationTalon.config_kF(0, liftPidController.getF());
  articulationTalon.config_kP(0, liftPidController.getP());
  articulationTalon.config_kI(0, liftPidController.getI());
  articulationTalon.config_kD(0, liftPidController.getD());

  }


  public void control(double input,double input2,double input3,double input4){
    articulationTalon.set(ControlMode.PercentOutput, input);
    rollerTalon.set(ControlMode.PercentOutput, input2);
    rollerTalonSlave.set(ControlMode.PercentOutput,-input2);
    dropTalon.set(ControlMode.PercentOutput, input3-input4);
  }

  
  public void articulate(double input){
    input =  Math.sin(input);
    double peakVelocity = 600;
    articulationTalon.set(ControlMode.Velocity, input*peakVelocity);
  }


  public boolean isZero(double input){
    if(input > 0.01){
      return true;
    }else{
      return false;
    }
  }

  @Override
  public void periodic() {
SmartDashboard.putNumber("articulation VELOCITY", articulationTalon.getSelectedSensorVelocity());
  }



 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   
    setDefaultCommand(new ArticulationCommand());
  }
}
