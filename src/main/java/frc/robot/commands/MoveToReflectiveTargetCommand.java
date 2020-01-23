package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem.ledMode;

/**
 * Moves the robot to a reflective target, within a specified distance from the target.
 *
 * @author Dowland Aiello
 **/
public abstract class MoveToReflectiveTargetCommand extends Command {
    /* The proportional value for the command's PID loop. This is the amount
     * that the input error value is multiplied by on each call. */
    public final double kP;

    /* The tolerance to error in this command's PID loop. This is the amount that
     * the error vlaue can deviate from the target value. */
    public final double errorTolerance;

    /* The desired distance on the z axis from the target. */
    public final double targetDistanceZ;

    /**
     * Initializes a new MoveToReflectiveTarget command.
     * This subsystem needs access to the vision and drive subsystems.
     **/
    public MoveToReflectiveTargetCommand(double kP, double errorTolerance, double targetDistanceZ) {
        // Set values for the PID loop embedded in this command
        this.kP = kP;
        this.errorTolerance = errorTolerance;

        // The robot must be `targetDistanceZ` away from the target
        this.targetDistanceZ = targetDistanceZ;

        // Make sure the limelight is always on
        Robot.m_visionsubsystem.setLEDMode(ledMode.ON);

        // Use the 0th pipeline preset
        Robot.m_visionsubsystem.setPipeline(0);

        requires(Robot.m_visionsubsystem);
        requires(Robot.m_drivesubsystem);
    }

    /**
     * An event called in the robot's main event loop aiding in the movement to a desired reflective target.
     **/
    @Override
    protected void execute() {
        // If there's no target, we don't need to do anything
        if (!Robot.m_visionsubsystem.hasTarget()) {
            return;
        }

        // Get the offset by which we need to move
        double offsetX = Robot.m_visionsubsystem.tx();
        double offsetY = Robot.m_visionsubsystem.ty();

        // Check if we need to correct for X at all
        if (Math.abs(offsetX) > errorTolerance) {
            // Drive to correct for the X
            Robot.m_drivesubsystem.rhinoDrive(kP / offsetX, -kP / offsetX);
        } else if (Math.abs(offsetY) > errorTolerance) {
            Robot.m_drivesubsystem.rhinoDrive(kP / offsetX, kP / offsetX);
        }
    }
}
