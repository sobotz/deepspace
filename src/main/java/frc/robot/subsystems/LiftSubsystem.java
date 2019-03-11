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
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.LiftCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX liftTalon;
  public TalonSRX liftTalonSlave;
  private SlotConfiguration liftSlotConfiguration1 = new SlotConfiguration();
  private int Talon_PID_TIMEOUT = 30;
  private PIDController liftPidController;

  public LiftSubsystem() {

    liftTalon = new TalonSRX(RobotMap.liftMotor);
    liftTalonSlave = new TalonSRX(RobotMap.liftMotorSlave);
    liftTalon.configFactoryDefault();
    liftTalonSlave.configFactoryDefault();
    liftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    liftTalon.setInverted(true);
    liftTalon.configPeakOutputReverse(-0.3);
    liftTalon.configPeakOutputForward(0.3);
    liftTalonSlave.follow(liftTalon);
    liftTalonSlave.setInverted(true);

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
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LIFT UNITs", (liftTalon.getSelectedSensorPosition() / 4096.0));
    SmartDashboard.putNumber("RAW UNITs", (liftTalon.getSelectedSensorPosition()));
    SmartDashboard.putNumber("LIFT ENCODER POSITION", talonUnitsToInches());
    SmartDashboard.putNumber("INCHES TO  ENCODER POSITION", inchesToTalonUnits(talonUnitsToInches()));
    SmartDashboard.putNumber("LIFT PID ERROR", liftTalon.getClosedLoopError());
    SmartDashboard.putBoolean("LIMIT REVERSE SWITCH ", liftTalon.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("LIMIT FORWARD SWITCH ", liftTalon.getSensorCollection().isFwdLimitSwitchClosed());

    liftTalon.config_kF(0, liftPidController.getF());
    liftTalon.config_kP(0, liftPidController.getP());
    liftTalon.config_kI(0, liftPidController.getI());
    liftTalon.config_kD(0, liftPidController.getD());
  }

  public void control(double output) {
    liftTalon.set(ControlMode.PercentOutput, output * -1);
  }

  // 12 inches -> 45 rotation #from the belly pan
  public void goTo(int position) {
    liftTalon.set(ControlMode.Position, inchesToTalonUnits(position));
    SmartDashboard.putNumber("LIFT PID TARGET", liftTalon.getClosedLoopTarget());
  }

  public double inchesToTalonUnits(double position) {
    return 3.75 * 4096 * (constrain(position - 2));
  }

  public static double constrain(double position) {
    if (position < 0) {
      return 0;
    }
    return position;
  }

  public void reset() {
    liftTalon.setSelectedSensorPosition(0);
  }
  

  //velocity is in/s
  public double talonUitsToVelocity(double speed) {
    return inchesToTalonUnits(speed) / 10.0;
  }

  public double talonUnitsToInches() {
    return ((liftTalon.getSelectedSensorPosition() / 4096.0) / 3.75);
  }

  public double talonVelocityUnitsToNormal() {
    return ((liftTalon.getSelectedSensorVelocity()/ 4096.0) / 3.75) * 10;
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new IntakeOpenCommand());

    setDefaultCommand(new LiftCommand());
  }
}
