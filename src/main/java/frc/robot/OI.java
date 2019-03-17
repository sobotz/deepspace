/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.autonomous.PathL2C4;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ShiftGearCommand;
import frc.robot.commands.liftGotoCommand;

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
   
    JoystickButton bOperator = new JoystickButton(operatorJoystick, 2);
    JoystickButton xOperator = new JoystickButton(operatorJoystick, 3);
    JoystickButton yOperator = new JoystickButton(operatorJoystick, 4);

    bOperator.toggleWhenPressed(new liftGotoCommand(12));
    xOperator.toggleWhenPressed(new liftGotoCommand(24));
    yOperator.toggleWhenPressed(new liftGotoCommand(36));
    aOperator.toggleWhenPressed(new liftGotoCommand(0));

    JoystickButton gearShiftButton = new JoystickButton(driverJoystick, 1);
    gearShiftButton.toggleWhenPressed(new ShiftGearCommand());

  }



}