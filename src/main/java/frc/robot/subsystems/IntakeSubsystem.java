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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class IntakeSubsystem extends Subsystem {
  TalonSRX articulationTalon;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX rollerTalon;
  public TalonSRX rollerTalonSlave;
  TalonSRX wristTalon;

  private double armsMaxPosition = -2200;

  private double iPosition = 100;
  private double currentArmPosition = 0;

  private double wristMaxPosition = 2000;

  private double currentWristPosition = 0;
  private double wristIPosition = 5000;

  public IntakeSubsystem() {
    articulationTalon = new TalonSRX(RobotMap.articulationMotor);

    rollerTalon = new TalonSRX(RobotMap.rollerMotor);
    rollerTalonSlave = new TalonSRX(RobotMap.rollerMotorSlave);

    wristTalon = new TalonSRX(RobotMap.wristTalon);

    articulationTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

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
    articulationTalon.configAllowableClosedloopError(0, 100);

    articulationTalon.setInverted(true);

    articulationTalon.config_kF(0, 1);
    articulationTalon.config_kP(0, 1.5);
    articulationTalon.config_kI(0, 0);
    articulationTalon.config_kD(0, 0);

    wristTalon.setSensorPhase(false);
    wristTalon.setNeutralMode(NeutralMode.Brake);

    wristTalon.configReverseSoftLimitThreshold(-190000);
    wristTalon.configReverseSoftLimitEnable(true);

    wristTalon.configMotionCruiseVelocity(1000, 30);
    wristTalon.configMotionAcceleration(1000, 30);
    wristTalon.configAllowableClosedloopError(0, 5000);

    wristTalon.config_kF(0, 0.1);
    wristTalon.config_kP(0, 0.05);
    wristTalon.config_kI(0, 0);
    wristTalon.config_kD(0, 0);

    reset();
  }

  public void control(double input, double rollerInput, double wristUp, double wristDown) {
    rollerTalon.set(ControlMode.PercentOutput, rollerInput * -1);
    rollerTalonSlave.set(ControlMode.PercentOutput, rollerInput);
    wristTalon.set(ControlMode.PercentOutput, wristUp - wristDown);

    articulateArms(input);
  }

  public void articulateArms(double input) {
    if (Math.abs(input) > 0.01) {
      if (input < 0) {
        if (currentArmPosition <= 0) {
          currentArmPosition += iPosition;
        }
        articulationTalon.set(ControlMode.MotionMagic, currentArmPosition);
      } else if (input > 0) {
        if (currentArmPosition >= armsMaxPosition) {
          currentArmPosition -= iPosition;
        }
        articulationTalon.set(ControlMode.MotionMagic, currentArmPosition);
      }
    } else {
      articulationTalon.set(ControlMode.MotionMagic, currentArmPosition);
    }
  }

  public void articulateWrist(double up, double down) {
    if (up > 0.01) {
      if (currentWristPosition <= 0) {
        currentWristPosition += wristIPosition;
      }
      wristTalon.set(ControlMode.MotionMagic, currentWristPosition);
    } else if (down > 0.01) {
      if (currentWristPosition >= wristMaxPosition) {
        currentWristPosition -= wristIPosition;
      }
      wristTalon.set(ControlMode.MotionMagic, currentWristPosition);
    } else {
      wristTalon.set(ControlMode.MotionMagic, currentWristPosition);
    }
  }

  public boolean isZero(double input) {
    if (input > 0.01) {
      return true;
    } else {
      return false;
    }
  }

  public void reset() {
    articulationTalon.setSelectedSensorPosition(0);
    wristTalon.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArticulationCommand());
  }
}
