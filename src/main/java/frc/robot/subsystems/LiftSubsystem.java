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
import frc.robot.commands.LiftCommand;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX liftTalon;
  public TalonSRX liftTalonSlave;
  private PIDController liftPidController;

  private ArrayList<Integer> velocities = new ArrayList<Integer>();
  private Timer recordPeakVelocityTimer = new Timer();
  private boolean recordPeakVelocityStarted = false;
  private int peakVelocity = 0;

  public LiftSubsystem() {

    liftTalon = new TalonSRX(RobotMap.liftMotor);
    liftTalonSlave = new TalonSRX(RobotMap.liftMotorSlave);
    liftTalon.configFactoryDefault();
    liftTalonSlave.configFactoryDefault();
    liftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    liftTalon.setInverted(true);
    liftTalon.configPeakOutputReverse(-0.65);
    liftTalon.configPeakOutputForward(0.65);
    liftTalonSlave.follow(liftTalon);
    liftTalonSlave.setInverted(true);

    liftTalon.setNeutralMode(NeutralMode.Brake);
    liftTalonSlave.setNeutralMode(NeutralMode.Brake);

   /* liftTalon.configForwardSoftLimitThreshold((int)inchesToTalonUnits(60));
    liftTalon.configReverseSoftLimitThreshold(0);

    liftTalon.configForwardSoftLimitEnable(true);
    liftTalon.configReverseSoftLimitEnable(true);
    */

    liftTalon.config_kF(0, 0.05);
    liftTalon.config_kP(0, 0.02);
    liftTalon.config_kI(0, 0.0);
    liftTalon.config_kD(0, 0.0);

    /// THIS IS A DUMMY Object !!! NEEDED only for testing !!!!!!!
    liftPidController = new PIDController(0, 0, 0, new PIDSource() {

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
    SmartDashboard.putData("LIFT PID", liftPidController);
    liftTalon.configMotionCruiseVelocity((int) velocityToTalonVelocity(36), 30);
    liftTalon.configMotionAcceleration((int) velocityToTalonVelocity(36), 30);

    liftTalon.configClearPositionOnLimitR(true, 30);
  }

  @Override
  public void periodic() {
    /*
    SmartDashboard.putNumber("TALON VELOCITY", talonVelocityToNormal());
    SmartDashboard.putNumber("INCHES TO  ENCODER POSITION", liftTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("LIFT PID ERROR", liftTalon.getClosedLoopError());
    SmartDashboard.putNumber("INCHES", talonUnitsToInches());
    */
    liftTalon.config_kF(0, liftPidController.getF());
    liftTalon.config_kP(0, liftPidController.getP());
    liftTalon.config_kI(0, liftPidController.getI());
    liftTalon.config_kD(0, liftPidController.getD());

    if (liftTalon.getSensorCollection().isRevLimitSwitchClosed()) {
      // reset();
    }
  }

  public void control(boolean left,boolean right) {

    double upSpeed = 0.4;
    double downSpeed = -0.4;

    if(right){
      liftTalon.set(ControlMode.PercentOutput, upSpeed);
    }else if(left){
      liftTalon.set(ControlMode.PercentOutput, downSpeed);
    }else{
      liftTalon.set(ControlMode.MotionMagic, liftTalon.getSelectedSensorPosition());
    }
    
    SmartDashboard.putNumber("TALON RAW VELOCITY", liftTalon.getSelectedSensorVelocity());
  }

  public void goTo(double position) {
    liftTalon.set(ControlMode.MotionMagic, inchesToTalonUnits(position));
    SmartDashboard.putNumber("LIFT PID TARGET", liftTalon.getClosedLoopTarget());
  }

  public boolean onTarget(int margin) {
    liftTalon.configAllowableClosedloopError(0, (int) inchesToTalonUnits(margin), 30);
    if (talonUnitsToInches((double) Math.abs(liftTalon.getClosedLoopError())) <= margin) {
      return true;
    } else {
      return false;
    }
  }

  public static double positionConstraint(double position) {
    if (position < 0) {
      return 0;
    }
    return position;
  }

  public void reset() {
    liftTalon.setSelectedSensorPosition(0);
    recordPeakVelocityStarted = true;
    peakVelocity =0;
  }

  // velocity is in/s

  public double inchesToTalonUnits(double position) {
    return 3.83 * 4096 * (positionConstraint(position - 10));
  }

  public double talonUnitsToInches() {
    return ((liftTalon.getSelectedSensorPosition() / 4096.0) / 3.83);
  }

  public double talonUnitsToInches(double units) {
    return ((units / 4096.0) / 3.83);
  }

  public double velocityToTalonVelocity(double speed) {
    return (3.83 * 4096 * speed) / 10.0;
  }

  public double talonVelocityToNormal() {
    return talonUnitsToInches(liftTalon.getSelectedSensorVelocity()) * 10;
  }

  public void recordPeakVelocity() {
    if (recordPeakVelocityStarted) {
      recordPeakVelocityTimer.start();
      if (recordPeakVelocityTimer.get() < 10) {
        if (peakVelocity < liftTalon.getSelectedSensorVelocity()/4096) {
          peakVelocity = liftTalon.getSelectedSensorVelocity()/4096;
        }
      } else {
        SmartDashboard.putNumber("PEAK VELOCITY", peakVelocity);
        recordPeakVelocityStarted = false;
      }
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LiftCommand());
  }
}
