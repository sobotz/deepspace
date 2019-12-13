/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.autonomous.*;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ArticulationCommand;
import frc.robot.commands.DeliverHatchCommand;
import frc.robot.commands.LegsCommand;
import frc.robot.commands.ShiftGearCommand;
import frc.robot.commands.liftGotoCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  /* 
   * Various heights for the lift subsystem
   */

  /* An old height (no longer in use) */
  public int HLVL1 = 18; // NOT IN USE

  /* The only cargo ship height */
  public int CSC1 = 38; // good

  /* The rocket ship heights */
  public int RSH2 = 40; // good
  public int RSH3 = 65; // good
  public int RSC1 = 27;
  public int RSC2 = 50; // good
  public int RSC3 = 67; // good

  /* An old height (no longer in use) */
  public int FSH1 = 19; // good (pneu)

  /* Zero */
  public int ZERO = 0; // good

  public Joystick driverJoystick = new Joystick(0);
  public Joystick secondaryDriverJoystick = new Joystick(1);
  public Joystick operatorJoystick = new Joystick(2);
  public Joystick m_buttonbox = new Joystick(3);

  public OI() {
    JoystickButton aOperator = new JoystickButton(operatorJoystick, 1);
    JoystickButton startOperator = new JoystickButton(operatorJoystick, 8);
    JoystickButton backOperator = new JoystickButton(operatorJoystick, 7);

    aOperator.toggleWhenPressed(new DeliverHatchCommand());
    startOperator.whenPressed(new liftGotoCommand(HLVL1));
    backOperator.whenPressed(new liftGotoCommand(ZERO));

    JoystickButton buttonbox1 = new JoystickButton(m_buttonbox, 1);
    JoystickButton buttonbox2 = new JoystickButton(m_buttonbox, 2);
    JoystickButton buttonbox3 = new JoystickButton(m_buttonbox, 3);
    JoystickButton buttonbox4 = new JoystickButton(m_buttonbox, 4);
    JoystickButton buttonbox5 = new JoystickButton(m_buttonbox, 5);
    JoystickButton buttonbox6 = new JoystickButton(m_buttonbox, 6);
    JoystickButton buttonbox7 = new JoystickButton(m_buttonbox, 7);
    JoystickButton buttonbox8 = new JoystickButton(m_buttonbox, 8);
    JoystickButton buttonbox9 = new JoystickButton(m_buttonbox, 9);

    buttonbox1.whenPressed(new liftGotoCommand(HLVL1));
    buttonbox2.whenPressed(new liftGotoCommand(RSH2));
    buttonbox3.whenPressed(new liftGotoCommand(RSH3));
    buttonbox4.whenPressed(new liftGotoCommand(CSC1));
    buttonbox5.whenPressed(new liftGotoCommand(RSC1));
    buttonbox6.whenPressed(new liftGotoCommand(RSC2));
    buttonbox7.whenPressed(new liftGotoCommand(RSC3));
    buttonbox8.whenPressed(new liftGotoCommand(FSH1));
    buttonbox9.whenPressed(new liftGotoCommand(ZERO));

    JoystickButton gearShiftButton = new JoystickButton(driverJoystick, 1);
    gearShiftButton.toggleWhenPressed(new ShiftGearCommand());

    JoystickButton expandsFrontlegs = new JoystickButton(driverJoystick, 7);
    expandsFrontlegs.whenPressed(new LegsCommand(7));

    JoystickButton retractsFrontlegs = new JoystickButton(driverJoystick, 8);
    retractsFrontlegs.whenPressed(new LegsCommand(8));

    JoystickButton retractsBackLeg = new JoystickButton(driverJoystick, 9);
    retractsBackLeg.whenPressed(new LegsCommand(9));

    JoystickButton expandsBackLeg = new JoystickButton(driverJoystick, 10);
    expandsBackLeg.whenPressed(new LegsCommand(10));

  }

}
