/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ShiftGearCommand;
import frc.robot.commands.liftGotoCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  // Create joystick instances
  public Joystick m_driverjoystick = new Joystick(0);
  public Joystick m_operatorjoystick = new Joystick(1);
  public Joystick m_buttonbox = new Joystick(2);

  public OI() {

    // Create buttons on the operator's GamePad
    JoystickButton aOperator = new JoystickButton(m_operatorjoystick, 1);
    JoystickButton bOperator = new JoystickButton(m_operatorjoystick, 2);
    JoystickButton xOperator = new JoystickButton(m_operatorjoystick, 3);
    JoystickButton yOperator = new JoystickButton(m_operatorjoystick, 4);

    // Add commands to the buttons on the operator's GamePad
    bOperator.toggleWhenPressed(new liftGotoCommand(12));
    xOperator.toggleWhenPressed(new liftGotoCommand(24));
    yOperator.toggleWhenPressed(new liftGotoCommand(36));
    aOperator.toggleWhenPressed(new liftGotoCommand(0));

    // Add gear shifting to the trigger on the driver's joystick
    JoystickButton gearShiftButton = new JoystickButton(m_driverjoystick, 1);
    gearShiftButton.toggleWhenPressed(new ShiftGearCommand());

  }

}
