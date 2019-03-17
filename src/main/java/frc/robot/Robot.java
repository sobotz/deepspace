/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static DriveSubsystem m_drivesubsystem;
  public static IntakeSubsystem m_intake;
  public static LiftSubsystem m_lift;


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_drivesubsystem = new DriveSubsystem();
    m_intake = new IntakeSubsystem();
    m_lift = new LiftSubsystem();
    m_oi = new OI();
    // starting from L1
    m_chooser.addDefault("Path L1R1L (Pure Pursuit)", new PathL1R1L(true));
    m_chooser.addObject("Path L1R1L (Regular)", new PathL1R1L(false));
    m_chooser.addObject("Path L1C3 (Pure Pursuit)", new PathL1C3(true));
    m_chooser.addObject("Path L1C3 (Regular)", new PathL1C3(false));
    // starting from L2
    m_chooser.addObject("Path L2C3 (Pure Pursuit)", new PathL2C3(true));
    m_chooser.addObject("Path L2C3 (Regular)", new PathL2C3(false));
    m_chooser.addObject("Path L2C4 (Pure Pursuit)", new PathL2C4(true));
    m_chooser.addObject("Path L2C4 (Regular)", new PathL2C4(false));
    m_chooser.addObject("Path L2C5 (Pure Pursuit)", new PathL2C5(true));
    m_chooser.addObject("Path L2C5 (Regular)", new PathL2C5(false));
    m_chooser.addObject("Path L2C6 (Pure Pursuit)", new PathL2C6(true));
    m_chooser.addObject("Path L2C6 (Regular)", new PathL2C6(false));
    // starting from L3
    m_chooser.addObject("Path L3R1R (Pure Pursuit)", new PathL3R1R(true));
    m_chooser.addObject("Path L3R1R (Regular)", new PathL3R1R(false));
    m_chooser.addObject("Path L3C6 (Pure Pursuit)", new PathL3C6(true));
    m_chooser.addObject("Path L3C6 (Regular)", new PathL3C6(false));
    
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
