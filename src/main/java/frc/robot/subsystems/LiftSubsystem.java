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
import frc.robot.commands.LiftCommand;
import com.ctre.phoenix.motorcontrol.ControlMode;
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

  private double liftMaxPosition = inchesToTalonUnits(60);
  private double lastPosition = 0;

  private boolean liftControlSwitch = true;

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

    liftTalon.configForwardSoftLimitThreshold((int) inchesToTalonUnits(78)); // changed from 78

    liftTalon.configForwardSoftLimitEnable(true);

    liftTalon.config_kF(0, 0.03);
    liftTalon.config_kP(0, 0.01);
    liftTalon.config_kI(0, 0.0);
    liftTalon.config_kD(0, 0.0);

    onTarget(1);

    liftTalon.configMotionCruiseVelocity((int) velocityToTalonVelocity(40), 30);
    liftTalon.configMotionAcceleration((int) velocityToTalonVelocity(25), 30);

    liftTalon.configClearPositionOnLimitR(true, 30);
  }

  @Override
  public void periodic() {

  }

  public void control(boolean left, boolean right) {

    if (right) {
      if (lastPosition < liftMaxPosition) {
        liftTalon.set(ControlMode.MotionMagic, lastPosition);
        if (liftControlSwitch) {
          lastPosition = liftTalon.getSelectedSensorPosition() + 800;
          liftControlSwitch = false;

        } else {
          lastPosition += 800;
        }
      }
    } else if (left) {
      liftTalon.set(ControlMode.MotionMagic, lastPosition);
      if (lastPosition > -1500) {
        if (liftControlSwitch) {
          lastPosition = liftTalon.getSelectedSensorPosition() - 800;
          liftControlSwitch = false;
        } else {
          lastPosition -= 800;
        }
      }
    } else {
      liftTalon.set(ControlMode.MotionMagic, lastPosition);
    }

    SmartDashboard.putNumber("Lift max position", liftMaxPosition);
    SmartDashboard.putNumber("Lift current position", lastPosition);

  }

  public void goTo(double position) {
    liftTalon.set(ControlMode.MotionMagic, inchesToTalonUnits(position));
    lastPosition = liftTalon.getSelectedSensorPosition();
    liftControlSwitch = true;
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
    peakVelocity = 0;
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
        if (peakVelocity < liftTalon.getSelectedSensorVelocity() / 4096) {
          peakVelocity = liftTalon.getSelectedSensorVelocity() / 4096;
        }
      } else {
        recordPeakVelocityStarted = false;
      }
    }
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new LiftCommand());
  }
}
