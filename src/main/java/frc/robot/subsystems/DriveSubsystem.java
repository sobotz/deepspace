/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
    public WPI_TalonSRX frontLeftTalon, backLeftTalon, frontRightTalon, backRightTalon;

    public TalonSRX legsTalon;

    private double wheelDiameter = 6.25;

    PIDController talonsPIDTuner;
    SpeedControllerGroup m_left, m_right;
    public DifferentialDrive m_drive;
    private DoubleSolenoid gearShifter;
    private DoubleSolenoid hatchDelivery;
    private boolean gearShifterState = false;
    private boolean hatchDeliveryState = false;

    public final AHRS ahrs = new AHRS(SPI.Port.kMXP);;

    private static VisionSubsystem vision = new VisionSubsystem();
    public VisionState visionState = VisionState.SEEKING;

    private PIDController rotateToTargetPID;
    private double rkP = 0.05, rkI, rkD = 0.0;
    private double rotateToTargetOutput = 0.0;
    private RotateToTargetInput rotateToTargetInput;

    private PIDController driveToTargetPID;
    private double dkP = 0.05, dkI, dkD = 0.0;
    private double driveToTargetOutput = 0.0;
    private DriveToTargetInput driveToTargetInput;

    public static enum VisionState {
        HASTARGET, SEEKING
    }

    public static enum DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE {
        VISION, SENSOR, ANGLE
    }

    public DriveSubsystem() {
        gearShifter = new DoubleSolenoid(0, 1);
        hatchDelivery = new DoubleSolenoid(2, 3);
        rotateToTargetInput = new RotateToTargetInput();

        rotateToTargetPID = new PIDController(rkP, rkI, rkD, rotateToTargetInput, new RotateToTargetOutput());
        rotateToTargetPID.setOutputRange(-0.7, 0.7);
        SmartDashboard.putData("rotateToTargetPID", rotateToTargetPID);

        driveToTargetInput = new DriveToTargetInput();

        driveToTargetPID = new PIDController(dkP, dkI, dkD, driveToTargetInput, new DriveToTargetOutput());
        driveToTargetPID.setOutputRange(-0.7, 0.7);
        SmartDashboard.putData("driveToTargetPID", driveToTargetPID);

        frontLeftTalon = new WPI_TalonSRX(RobotMap.frontLeftMotor);
        backLeftTalon = new WPI_TalonSRX(RobotMap.backLeftMotor);
        frontRightTalon = new WPI_TalonSRX(RobotMap.frontRightMotor);
        backRightTalon = new WPI_TalonSRX(RobotMap.backRightMotor);

        frontRightTalon.setInverted(true);
        backRightTalon.setInverted(true);
        frontLeftTalon.configFactoryDefault();
        backLeftTalon.configFactoryDefault();
        frontRightTalon.configFactoryDefault();
        backRightTalon.configFactoryDefault();

        frontLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        frontLeftTalon.setSensorPhase(true);

        frontRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        frontRightTalon.setSensorPhase(true);

        frontLeftTalon.configPeakOutputReverse(-1);
        frontLeftTalon.configPeakOutputForward(1);
        frontRightTalon.configPeakOutputReverse(-1);
        frontRightTalon.configPeakOutputForward(1);

        talonsPIDTuner = new PIDController(0, 0, 0, new DriveToTargetInput(), new DriveToTargetOutput());

        m_left = new SpeedControllerGroup(frontLeftTalon, backLeftTalon);
        m_right = new SpeedControllerGroup(frontRightTalon, backRightTalon);

        reset();

        frontLeftTalon.config_kF(0, 0.02);
        frontLeftTalon.config_kP(0, 0.5);
        frontLeftTalon.config_kI(0, 0);
        frontLeftTalon.config_kD(0, 0);

        frontRightTalon.config_kF(0, 0.02);
        frontRightTalon.config_kP(0, 0.5);
        frontRightTalon.config_kI(0, 0);
        frontRightTalon.config_kD(0, 0);

        legsTalon = new TalonSRX(RobotMap.legsMotor);

        legsTalon.configFactoryDefault();

        legsTalon.config_kF(0, 0.02);
        legsTalon.config_kP(0, 0.5);
        legsTalon.config_kI(0, 0);
        legsTalon.config_kD(0, 0);

        legsTalon.setSelectedSensorPosition(0);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new DriveCommand());
    }

    @Override
    public void periodic() {
        seekTarget();
    }

    public void legsControl(int input) {
        if (input == 0) {
            legsTalon.set(ControlMode.PercentOutput, -0.2);
        } else if (input == 180) {

            legsTalon.set(ControlMode.PercentOutput, 0.2);
        } else {
            legsTalon.set(ControlMode.PercentOutput, 0);
        }

    }

    /**
     * Drives the robot using the provided left and right controller speeds.
     *
     * @param speed1 the speed of the left controller
     * @param speed2 the speed of the right controller
     **/
    public void rhinoDrive(double speed1, double speed2) {
        // Apply a logarithmic aplifier to the left and right speeds
        speed1 = normalizeVelocity(speed1);
        speed2 = normalizeVelocity(speed2);

        // The left FULL_FWD button
        if (Robot.m_oi.driverJoystick.getRawButton(11)) {
            frontLeftTalon.set(ControlMode.PercentOutput, -1);
            backLeftTalon.follow(frontLeftTalon);

            return;
        }

        // The right FULL_FWD button
        if (Robot.m_oi.secondaryDriverJoystick.getRawButton(11)) {
            frontRightTalon.set(ControlMode.PercentOutput, -1);
            backRightTalon.follow(frontRightTalon);

            return;
        }

        // The left FULL_REV button
        if (Robot.m_oi.driverJoystick.getRawButton(11)) {
            frontLeftTalon.set(ControlMode.PercentOutput, 1);
            backLeftTalon.follow(frontLeftTalon);

            return;
        }

        // The right FULL_REV button
        if (Robot.m_oi.secondaryDriverJoystick.getRawButton(11)) {
            frontRightTalon.set(ControlMode.PercentOutput, 1);
            backRightTalon.follow(frontRightTalon);

            return;
        }

        frontLeftTalon.set(ControlMode.PercentOutput, -speed1, DemandType.ArbitraryFeedForward, 0);
        backLeftTalon.follow(frontLeftTalon);

        frontRightTalon.set(ControlMode.PercentOutput, -speed2, DemandType.ArbitraryFeedForward, 0);
        backRightTalon.follow(frontRightTalon);
    }

    public void manualDrive2(double speed, double rotation) {
        // Apply a logarithmic aplifier to the speed and rotation according to
        // SmartDashboard prefs
        speed = normalizeVelocity(speed);
        rotation = normalizeVelocity(rotation);

        boolean test = false;
        SmartDashboard.putBoolean("TEST", Robot.m_oi.driverJoystick.getRawButton(11));
        if (!Robot.m_oi.driverJoystick.getRawButton(11) & !Robot.m_oi.driverJoystick.getRawButton(12)) {
            test = true;
            frontLeftTalon.set(ControlMode.PercentOutput, -speed, DemandType.ArbitraryFeedForward, rotation);
            backLeftTalon.follow(frontLeftTalon);

            frontRightTalon.set(ControlMode.PercentOutput, -speed, DemandType.ArbitraryFeedForward, -rotation);
            backRightTalon.follow(frontRightTalon);
        }

        if (Robot.m_oi.driverJoystick.getRawButton(11)) {
            frontLeftTalon.set(ControlMode.PercentOutput, -1);
            frontRightTalon.set(ControlMode.PercentOutput, -1);
            backLeftTalon.follow(frontLeftTalon);
            backRightTalon.follow(frontRightTalon);
            test = true;
        }

        SmartDashboard.putBoolean("TEST2", test);

        if (Robot.m_oi.driverJoystick.getRawButton(12)) {

            frontLeftTalon.set(ControlMode.PercentOutput, 1);
            frontRightTalon.set(ControlMode.PercentOutput, 1);

            backLeftTalon.follow(frontLeftTalon);
            backRightTalon.follow(frontRightTalon);

        }

    }

    public void shiftGear() {
        if (gearShifterState) {
            gearShifter.set(DoubleSolenoid.Value.kForward);
            gearShifterState = false;
        } else {
            gearShifter.set(DoubleSolenoid.Value.kReverse);
            gearShifterState = true;
        }
    }

    public void deliverHatch() {
        if (hatchDeliveryState) {
            hatchDelivery.set(DoubleSolenoid.Value.kForward);
            hatchDeliveryState = false;
        } else {
            hatchDelivery.set(DoubleSolenoid.Value.kReverse);
            hatchDeliveryState = true;
        }
    }

    private void seekTarget() {
        if (vision.hasTarget()) {
            visionState = VisionState.HASTARGET;
        } else {
            visionState = VisionState.SEEKING;
        }
    }

    public boolean rotationCascadePID(double setPoint, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type, double minOutput,
            double maxOutput, int errorMargin) {
        rotateToTargetInput.setType(type);
        rotateToTargetPID.setAbsoluteTolerance(errorMargin);
        rotateToTargetPID.setSetpoint(setPoint);
        rotateToTargetPID.setOutputRange(minOutput, maxOutput);
        rotateToTargetPID.enable();
        return false;
    }

    public boolean rotationCascadePID(double setPoint, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type, int errorMargin) {
        rotateToTargetInput.setType(type);
        rotateToTargetPID.setAbsoluteTolerance(errorMargin);
        rotateToTargetPID.setSetpoint(setPoint);
        rotateToTargetPID.setOutputRange(-1, 1);
        rotateToTargetPID.enable();
        return false;
    }

    public boolean distanceCascadePID(double setPoint, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type, double minOutput,
            double maxOutput, int errorMargin) {
        rotateToTargetInput.setType(type);
        driveToTargetPID.setAbsoluteTolerance(errorMargin);
        driveToTargetPID.setSetpoint(setPoint);
        driveToTargetPID.setOutputRange(minOutput, maxOutput);
        driveToTargetPID.enable();
        return false;
    }

    public boolean distanceCascadePID(double setPoint, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type, int errorMargin) {
        rotateToTargetInput.setType(type);
        driveToTargetPID.setAbsoluteTolerance(errorMargin);
        driveToTargetPID.setSetpoint(setPoint);
        driveToTargetPID.setOutputRange(-1, 1);
        driveToTargetPID.enable();
        return false;
    }

    public boolean rotateToTarget(int errorMargin) {
        if (!rotateToTargetPID.isEnabled()) {
            rotationCascadePID(0, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION, errorMargin);
        }

        backLeftTalon.follow(frontLeftTalon);
        backRightTalon.follow(frontRightTalon);

        double rightTalonTargetVelocity = velocityToTalonVelocity(-rotateToTargetOutput * 12);
        double leftTalonTargetVelocity = velocityToTalonVelocity(rotateToTargetOutput * 12);
        SmartDashboard.putNumber("rightTalonTargetVelocity", rightTalonTargetVelocity);
        SmartDashboard.putNumber("leftTalonTargetVelocity", leftTalonTargetVelocity);
        frontLeftTalon.set(ControlMode.Velocity, leftTalonTargetVelocity);
        frontRightTalon.set(ControlMode.Velocity, rightTalonTargetVelocity);
        onTarget(frontLeftTalon, errorMargin);
        onTarget(frontRightTalon, errorMargin);
        return false;
    }

    public boolean rotateToTarget(double angle, int errorMargin) {
        if (!rotateToTargetPID.isEnabled()) {
            rotationCascadePID(angle, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION, errorMargin);
        }

        backLeftTalon.follow(frontLeftTalon);
        backRightTalon.follow(frontRightTalon);

        double rightTalonTargetVelocity = velocityToTalonVelocity(-rotateToTargetOutput * 12);
        double leftTalonTargetVelocity = velocityToTalonVelocity(rotateToTargetOutput * 12);
        SmartDashboard.putNumber("rightTalonTargetVelocity", rightTalonTargetVelocity);
        SmartDashboard.putNumber("leftTalonTargetVelocity", leftTalonTargetVelocity);
        frontLeftTalon.set(ControlMode.Velocity, leftTalonTargetVelocity);
        frontRightTalon.set(ControlMode.Velocity, rightTalonTargetVelocity);
        onTarget(frontLeftTalon, errorMargin);
        onTarget(frontRightTalon, errorMargin);
        return rotateToTargetPID.onTarget();
    }

    public boolean driveToTarget(int errorMargin) {
        if (!driveToTargetPID.isEnabled()) {
            distanceCascadePID(0, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION, errorMargin);
        }

        backLeftTalon.follow(frontLeftTalon);
        backRightTalon.follow(frontRightTalon);
        double rightOutput = (driveToTargetOutput + rotateToTargetOutput);
        double leftOutput = (driveToTargetOutput - rotateToTargetOutput);
        double rightTalonTargetVelocity = velocityToTalonVelocity(rightOutput * 12);
        double leftTalonTargetVelocity = velocityToTalonVelocity(leftOutput * 12);
        SmartDashboard.putNumber("rightTalonTargetVelocity", rightTalonTargetVelocity);
        SmartDashboard.putNumber("leftTalonTargetVelocity", leftTalonTargetVelocity);
        frontLeftTalon.set(ControlMode.Velocity, leftTalonTargetVelocity);
        frontRightTalon.set(ControlMode.Velocity, rightTalonTargetVelocity);
        onTarget(frontLeftTalon, errorMargin);
        onTarget(frontRightTalon, errorMargin);

        return driveToTargetPID.onTarget();
    }

    public boolean driveToTarget(double distance, int errorMargin) {
        frontLeftTalon.selectProfileSlot(0, 0);
        frontRightTalon.selectProfileSlot(0, 0);
        backLeftTalon.follow(frontLeftTalon);
        backRightTalon.follow(frontRightTalon);

        double targetPositionRotations = inchesToTalonUnits(distance);
        frontLeftTalon.set(ControlMode.Position, targetPositionRotations);
        frontRightTalon.set(ControlMode.Position, targetPositionRotations);

        return onTarget(frontLeftTalon, errorMargin) && onTarget(frontRightTalon, errorMargin);
    }

    public boolean driveRotateToTarget(int errorMargin) {
        if (!rotateToTargetPID.isEnabled()) {
            rotationCascadePID(0, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION, errorMargin);
        }

        if (!driveToTargetPID.isEnabled()) {
            distanceCascadePID(0, DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION, errorMargin);
        }

        backLeftTalon.follow(frontLeftTalon);
        backRightTalon.follow(frontRightTalon);
        double rightOutput = (driveToTargetOutput + rotateToTargetOutput);
        double leftOutput = (driveToTargetOutput - rotateToTargetOutput);
        double rightTalonTargetVelocity = velocityToTalonVelocity(rightOutput * 12);
        double leftTalonTargetVelocity = velocityToTalonVelocity(leftOutput * 12);
        SmartDashboard.putNumber("rightTalonTargetVelocity", rightTalonTargetVelocity);
        SmartDashboard.putNumber("leftTalonTargetVelocity", leftTalonTargetVelocity);
        frontLeftTalon.set(ControlMode.Velocity, leftTalonTargetVelocity);
        frontRightTalon.set(ControlMode.Velocity, rightTalonTargetVelocity);
        onTarget(frontLeftTalon, errorMargin);
        onTarget(frontRightTalon, errorMargin);

        return driveToTargetPID.onTarget();
    }

    public void PurePursuit(double targetLeft, double targetRight) {
        frontLeftTalon.selectProfileSlot(0, 0);
        frontRightTalon.selectProfileSlot(0, 0);
        double targetLeftVelocity = velocityToTalonVelocity(targetLeft);
        double targetRightVelocity = velocityToTalonVelocity(targetRight);
        SmartDashboard.putNumber("PURE PURSUIT VELOCITIES LEFT", targetLeftVelocity);
        SmartDashboard.putNumber("PURE PURSUIT VELOCITIES RIGHT", targetRightVelocity);
        frontLeftTalon.set(ControlMode.Velocity, targetLeftVelocity);
        frontRightTalon.set(ControlMode.Velocity, targetRightVelocity);
    }

    public void dashboard() {
        SmartDashboard.putNumber("L ENCODER position", frontLeftTalon.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("R ENCODER position", frontRightTalon.getSelectedSensorPosition(0));
        SmartDashboard.putBoolean("HAS TARGET", vision.hasTarget());
        SmartDashboard.putNumber("YAW", ahrs.getYaw());
        SmartDashboard.putNumber("TARGET DISTANCE", Robot.m_oi.driverJoystick.getRawAxis(3));
        SmartDashboard.putNumber("Talons left to inches", talonUnitsToInches(frontLeftTalon));
        SmartDashboard.putNumber("Talons right to inches", talonUnitsToInches(frontRightTalon));
        SmartDashboard.putBoolean("Talons left onTarget", onTarget(frontLeftTalon, 2));
        SmartDashboard.putBoolean("Talons rigth onTarget", onTarget(frontRightTalon, 2));
        SmartDashboard.putNumber("Talon left Error", frontLeftTalon.getClosedLoopError());
        SmartDashboard.putNumber("Talon right Error", frontRightTalon.getClosedLoopError());

        SmartDashboard.putNumber("Talon left Velocity", talonVelocityToNormal(frontLeftTalon));
        SmartDashboard.putNumber("Talon right Velocity", talonVelocityToNormal(frontRightTalon));
    }

    public void reset() {
        rotateToTargetPID.reset();
        driveToTargetPID.reset();
        ahrs.reset();
        frontLeftTalon.setSelectedSensorPosition(0);
        frontLeftTalon.stopMotor();
        frontRightTalon.setSelectedSensorPosition(0);
        frontRightTalon.stopMotor();
        Timer.delay(0.002);
    }

    class RotateToTargetInput implements PIDSource {
        private DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type = DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION;
        double input = 0.0;

        public RotateToTargetInput() {

        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {

        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public double pidGet() {

            if (type == DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION) {
                input = vision.tx() * -1;
            } else {
                input = ahrs.getYaw();
            }
            return input;
        }

        /**
         * @return the type
         */
        public DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE getType() {
            return type;
        }

        /**
         * @param type the type to set
         */
        public void setType(DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type) {
            this.type = type;
        }
    }

    class RotateToTargetOutput implements PIDOutput {
        @Override
        public void pidWrite(double output) {
            // Rino drive doesn't support rotation
            // rotateToTargetOutput = output;
        }

    }

    class DriveToTargetOutput implements PIDOutput {
        @Override
        public void pidWrite(double output) {
            driveToTargetOutput = output;
        }
    }

    class DriveToTargetInput implements PIDSource {
        DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type = DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION;
        double input = 0.0;

        public DriveToTargetInput() {
        }

        @Override
        public double pidGet() {
            if (type == DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.SENSOR) {
                input = 0.0;
            } else if (type == DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE.VISION) {
                input = vision.getTargetDistance();
            } else {
                input = vision.ty();
            }

            return input;
        }

        /**
         * @return the type
         */
        public DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE getType() {
            return type;
        }

        /**
         * @param type the type to set
         */
        public void setType(DRIVETRAIN_CONTROL_LOOP_INPUT_TYPE type) {
            this.type = type;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {

        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return null;
        }

    }

    public boolean onTarget(TalonSRX talon, int margin) {
        frontLeftTalon.configAllowableClosedloopError(0, (int) inchesToTalonUnits(margin), 30);
        frontRightTalon.configAllowableClosedloopError(0, (int) inchesToTalonUnits(margin), 30);
        if (talonUnitsToInches((double) Math.abs(talon.getClosedLoopError())) <= margin) {
            return true;
        } else {
            return false;
        }
    }

    public double talonUnitsToInches(TalonSRX talon) {
        return (talon.getSelectedSensorPosition() / 4096.0) * Math.PI * wheelDiameter;
    }

    public double talonUnitsToInches(double units) {
        return (units / 4096.0) * Math.PI * wheelDiameter;
    }

    public double inchesToTalonUnits(double inches) {
        return (inches / (Math.PI * wheelDiameter)) * 4096.0;
    }

    public double velocityToTalonVelocity(double speed) {
        return inchesToTalonUnits(speed) / 10.0;
    }

    public double talonVelocityToNormal(TalonSRX talon) {

        return talonUnitsToInches((double) talon.getSelectedSensorVelocity()) * 10;
    }

    public void setTalonPIDGains(double kF, double kP, double kI, double kD) {

        frontLeftTalon.config_kF(0, talonsPIDTuner.getF());
        frontLeftTalon.config_kP(0, talonsPIDTuner.getP());
        frontLeftTalon.config_kI(0, talonsPIDTuner.getI());
        frontLeftTalon.config_kD(0, talonsPIDTuner.getD());

        frontRightTalon.config_kF(0, talonsPIDTuner.getF());
        frontRightTalon.config_kP(0, talonsPIDTuner.getP());
        frontRightTalon.config_kI(0, talonsPIDTuner.getI());
        frontRightTalon.config_kD(0, talonsPIDTuner.getD());

    }

    /**
     * Determines whether or not this drive train uses rhino drive or legacy mode.
     * If the "RhinoEnabled" key has not been set in the SmartDashboard,
     * RhinoEnabled is assumed to be false.
     *
     * @return whether or not this drive train uses rhino drive
     **/
    public static boolean usesRhinoDrive() {
        // If the technician hasn't enabled rhino drive, keep it turned off
        return Robot.m_preferences.getBoolean("RhinoEnabled", false);
    }

    /**
     * Applies a logarithmic amplifier to the given velocity according to the
     * SmartDashboard's settings.
     *
     * @param d the velocity to normalize
     * @return the normalized velocity
     **/
    public static double normalizeVelocity(double d) {
        double normalized = Math.pow(Math.abs(d), Robot.m_preferences.getDouble("JoystickInputAmplificationFactor", 1)); // Normalize
                                                                                                                         // the
                                                                                                                         // inputted
                                                                                                                         // velocity

        return d < 0 ? (-1 * normalized) : normalized;
    }
}
