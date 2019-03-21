/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
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
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class IntakeSubsystem extends Subsystem {
  TalonSRX articulationTalon;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX rollerTalon;
  public TalonSRX rollerTalonSlave;
  TalonSRX wristTalon;

  private ArrayList<Integer> velocities = new ArrayList<Integer>();
  private Timer recordPeakVelocityTimer = new Timer();
  private boolean recordPeakVelocityStarted = false;
  private int peakVelocity = 0;

  private PIDController intakePidController;


  private double wristMaxPosition = 1000;
  private double wristPosition = 0;


  public IntakeSubsystem() {
    articulationTalon = new TalonSRX(RobotMap.articulationMotor);
    //dropTalon = new TalonSRX(RobotMap.dropMotor);

    rollerTalon = new TalonSRX(RobotMap.rollerMotor);
    rollerTalonSlave = new TalonSRX(RobotMap.rollerMotorSlave);


    wristTalon = new TalonSRX(RobotMap.wristTalon);

    articulationTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, 30);
  ///  dropTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	0, 30);

  articulationTalon.configFactoryDefault();
  rollerTalon.configFactoryDefault();
  rollerTalonSlave.configFactoryDefault();
  wristTalon.configFactoryDefault();

  articulationTalon.configPeakOutputReverse(-1);
  articulationTalon.configPeakOutputForward(1);

  rollerTalon.configPeakOutputReverse(-0.4);
  rollerTalon.configPeakOutputForward(0.4);

  wristTalon.configPeakOutputReverse(-0.3);
  wristTalon.configPeakOutputForward(0.3);

  articulationTalon.setSensorPhase(false);
  articulationTalon.setNeutralMode(NeutralMode.Brake);

  //articulationTalon.configForwardSoftLimitThreshold(1000);

  articulationTalon.configMotionCruiseVelocity(300, 30);
  articulationTalon.configMotionAcceleration(300, 30);


  intakePidController = new PIDController(0, 0, 0, new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public double pidGet() {
      return 0;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return null;
    }
  }, new PIDOutput() {

    @Override
    public void pidWrite(double output) {

    }
  });

  SmartDashboard.putData("INTAKE PID TUNER", intakePidController);
  }


  public void control(double input,double input2,double input3,double input4){
   articulationTalon.set(ControlMode.PercentOutput, input*-1);
    rollerTalon.set(ControlMode.PercentOutput, input2);
    rollerTalonSlave.set(ControlMode.PercentOutput,-input2);
    wristTalon.set(ControlMode.PercentOutput, input3-input4);

///  articulate(input);
  }

  
  public void articulateArms(double input){
   articulationTalon.set(ControlMode.MotionMagic,1000);
  }

  public void articulateWrist(){
    wristTalon.set(ControlMode.MotionMagic, demand);
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
    SmartDashboard.putNumber("articulation PID ERROR", articulationTalon.getClosedLoopError());
    SmartDashboard.putNumber("articulation ENCODER POSITION ", articulationTalon.getSelectedSensorPosition());

articulationTalon.config_kF(0, intakePidController.getF());
articulationTalon.config_kP(0, intakePidController.getP());
articulationTalon.config_kI(0, intakePidController.getI());
articulationTalon.config_kD(0, intakePidController.getD());

  }



 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   
    setDefaultCommand(new ArticulationCommand());
  }
}
