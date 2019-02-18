/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShiftGearCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {


  public Joystick driverJoystick = new Joystick(0);
  public Joystick operatorJoystick = new Joystick(1);
  public Joystick m_buttonbox = new Joystick(2);

  public OI(){
    JoystickButton aOperator = new JoystickButton(operatorJoystick, 1);
    aOperator.toggleWhenPressed(new AlignCommand());


    JoystickButton gearShiftButton = new JoystickButton(driverJoystick, 1);
    //gearShiftButton.toggleWhenPressed(new ShiftGearCommand());

  }



}