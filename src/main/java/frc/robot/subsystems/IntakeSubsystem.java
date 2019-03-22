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
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ArticulationCommand;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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


  private double armsMaxPosition = -2200;
  private double armLastPosition = 0;

  private double p_position = 0;

  private double POSITION = 0.0;

  private double iPosition = 80;
  private double currentArmPosition = 0;


  private double wristMaxPosition = 2000;
  private double wristLastPosition = 0;

  private double currentWristPosition  = 0;
  private double wristIPosition  = 50;


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

  articulationTalon.configPeakOutputReverse(-0.3);
  articulationTalon.configPeakOutputForward(0.3);

  rollerTalon.configPeakOutputReverse(-0.4);
  rollerTalon.configPeakOutputForward(0.4);

  wristTalon.configPeakOutputReverse(-0.3);
  wristTalon.configPeakOutputForward(0.3);

  articulationTalon.setSensorPhase(false);
  articulationTalon.setNeutralMode(NeutralMode.Brake);

  articulationTalon.configReverseSoftLimitThreshold(-2200);
  articulationTalon.configReverseSoftLimitEnable(true);

  articulationTalon.configMotionCruiseVelocity(1000, 30);
  articulationTalon.configMotionAcceleration(1000, 30);
  articulationTalon.configAllowableClosedloopError(0, 5);

  articulationTalon.setInverted(true);

  articulationTalon.config_kF(0, 2);
  articulationTalon.config_kP(0, 3);
  articulationTalon.config_kI(0, 0);
  articulationTalon.config_kD(0, 0);

  wristTalon.setSensorPhase(false);
  wristTalon.setNeutralMode(NeutralMode.Brake);

  wristTalon.configReverseSoftLimitThreshold(-2200);
  wristTalon.configReverseSoftLimitEnable(true);

  wristTalon.configMotionCruiseVelocity(1000, 30);
  wristTalon.configMotionAcceleration(1000, 30);
  wristTalon.configAllowableClosedloopError(0, 5);

  wristTalon.setInverted(true);

  wristTalon.config_kF(0, 1);
  wristTalon.config_kP(0, 1.5);
  wristTalon.config_kI(0, 0);
  wristTalon.config_kD(0, 0);

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

 // SmartDashboard.putData("INTAKE PID TUNER", intakePidController);
  armLastPosition = articulationTalon.getSelectedSensorPosition();
  POSITION = articulationTalon.getSelectedSensorPosition();
  reset();
  }


  public void control(double input,double rollerInput,double wristUp,double wristDown){
  /// articulationTalon.set(ControlMode.PercentOutput, input*-1);
    rollerTalon.set(ControlMode.PercentOutput, rollerInput);
    rollerTalonSlave.set(ControlMode.PercentOutput,-rollerInput);
    wristTalon.set(ControlMode.PercentOutput, wristUp-wristDown);

    articulateArms(input);

    //articulateWrist(wristUp, wristDown);
  }

  
  /*public void articulateArms(double input){
   articulationTalon.set(ControlMode.MotionMagic,input);
  }*/

  public void articulateArms(double input){
      if(input < 0.01){
        if(currentArmPosition <= 0){
          currentArmPosition += iPosition;
        }
        articulationTalon.set(ControlMode.MotionMagic,currentArmPosition);
      }else if(input > 0.01){
        if(currentArmPosition >= armsMaxPosition){
          currentArmPosition -= iPosition;
        }
        articulationTalon.set(ControlMode.MotionMagic,currentArmPosition);
      }else{
        articulationTalon.set(ControlMode.MotionMagic,currentArmPosition);
      }

    //SmartDashboard.putNumber("ARTICULATION POV", Robot.m_oi.operatorJoystick.getPOV());
  }

  public void articulateWrist(double up,double down){
    if(up > 0.01){
      if(currentWristPosition <= 0){
        currentWristPosition += wristIPosition;
      }
      wristTalon.set(ControlMode.MotionMagic,currentWristPosition);
    }else if(down > 0.01){
      if(currentWristPosition >= wristMaxPosition){
        currentWristPosition -= wristIPosition;
      }
      wristTalon.set(ControlMode.MotionMagic,currentWristPosition);
    }else{
      wristTalon.set(ControlMode.MotionMagic,currentWristPosition);
    }
  }



  public boolean isZero(double input){
    if(input > 0.01){
      return true;
    }else{
      return false;
    }
  }

  public void reset(){
    articulationTalon.setSelectedSensorPosition(0);
    wristTalon.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
   // SmartDashboard.putNumber("articulation VELOCITY", articulationTalon.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("articulation PID ERROR", articulationTalon.getClosedLoopError());
  
  
    SmartDashboard.putNumber("Articulation  POSITION ", articulationTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("WRIST POSITION", wristTalon.getSelectedSensorPosition());
  }



 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   
    setDefaultCommand(new ArticulationCommand());
  }
}
