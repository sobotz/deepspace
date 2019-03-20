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
  public int HLVL1 = 19;
  public int CSC1 = 39;
  public int RSH2 = 47;
  public int RSH3 = 75;
  public int RSC1 = 27;
  public int RSC2 = 56;
  public int RSC3 = 78;
  public int FSC1 = 44;
  public int ZERO = 0;

  public Joystick driverJoystick = new Joystick(0);
  public Joystick operatorJoystick = new Joystick(1);
  public Joystick m_buttonbox = new Joystick(2);

  public OI() {
    JoystickButton aOperator = new JoystickButton(operatorJoystick, 1);
   
    JoystickButton bOperator = new JoystickButton(operatorJoystick, 2);
    JoystickButton xOperator = new JoystickButton(operatorJoystick, 3);
    JoystickButton yOperator = new JoystickButton(operatorJoystick, 4);
    JoystickButton buttonbox1 = new JoystickButton(m_buttonbox, 1);
    JoystickButton buttonbox2 = new JoystickButton(m_buttonbox, 2);
    JoystickButton buttonbox3 = new JoystickButton(m_buttonbox, 3);
    JoystickButton buttonbox4 = new JoystickButton(m_buttonbox, 4);
    JoystickButton buttonbox5 = new JoystickButton(m_buttonbox, 5);
    JoystickButton buttonbox6 = new JoystickButton(m_buttonbox, 6);
    JoystickButton buttonbox7 = new JoystickButton(m_buttonbox, 7);
    JoystickButton buttonbox8 = new JoystickButton(m_buttonbox, 8);
    JoystickButton buttonbox9 = new JoystickButton(m_buttonbox, 9);

   /* bOperator.toggleWhenPressed(new liftGotoCommand(12));
    xOperator.toggleWhenPressed(new liftGotoCommand(24));
    yOperator.toggleWhenPressed(new liftGotoCommand(36));
    aOperator.toggleWhenPressed(new liftGotoCommand(0));
    */
    
    buttonbox1.whenPressed(new liftGotoCommand(HLVL1));
    buttonbox2.whenPressed(new liftGotoCommand(CSC1));
    buttonbox3.whenPressed(new liftGotoCommand(RSH2));
    buttonbox4.whenPressed(new liftGotoCommand(RSH3));
    buttonbox5.whenPressed(new liftGotoCommand(RSC1));
    buttonbox6.whenPressed(new liftGotoCommand(RSC2));
    buttonbox7.whenPressed(new liftGotoCommand(RSC3));
    buttonbox8.whenPressed(new liftGotoCommand(FSC1));
    buttonbox9.whenPressed(new liftGotoCommand(ZERO));

    JoystickButton gearShiftButton = new JoystickButton(driverJoystick, 1);
    gearShiftButton.toggleWhenPressed(new ShiftGearCommand());

  }



}
