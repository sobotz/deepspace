/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.LiftCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX liftTalon;
  public TalonSRX liftTalonSlave;


  public LiftSubsystem() {
    liftTalon = new TalonSRX(RobotMap.liftMotor);
    liftTalonSlave = new TalonSRX(RobotMap.liftMotorSlave);
    liftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,	0, 30);
    liftTalon.setSensorPhase(true);
    liftTalon.configPeakOutputReverse(-0.3);
    liftTalon.configPeakOutputForward(0.3);
    liftTalonSlave.follow(liftTalon);
  }



    public double talonUnitsToInches(TalonSRX talon) {
        return (talon.getSelectedSensorPosition() / 4096.0);
    }


    @Override
    public void periodic() {
      SmartDashboard.putNumber("LIFT ENCODER POSITION", liftTalon.getSelectedSensorPosition());
    }


    public void control(double output){
      liftTalon.set(ControlMode.PercentOutput, output);
    }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new IntakeOpenCommand());

    setDefaultCommand(new LiftCommand());
  }
}
