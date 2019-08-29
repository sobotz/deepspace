package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem {
    static NetworkTable table;
    static NetworkTableInstance NI;

    private double cameraHeight = 46;
    private double targetHeight = 0.0;
    private double cameraAngle = 73.5;

    public enum ledMode {
        DEFAULT, BLINK, ON, OFF
    }

    public enum camMode {
        VISIONPROCESSING, DRIVERCAMERA
    }

    public enum streamMode {
        STANDARD, PiPMAIN, PiPSECONDARY
    }

    public enum TargetHeight {
        BALL, HATCH_PANEL
    }

    public enum DistanceType {
        REAL, ANGLE
    }

    public VisionSubsystem() {
        NI = NetworkTableInstance.getDefault();
        table = NI.getTable("limelight");
        setTargetHeight(TargetHeight.HATCH_PANEL);
        setCamDMode(camMode.DRIVERCAMERA);
    }

    public boolean hasTarget() {
        if (table.getEntry("tv").getDouble(0) > 0.0) {
            return true;
        } else {
            return false;
        }
    }

    public double getTargetDistance() {
        double d = 0.0;
        if (hasTarget())
            d = Math.tan(Math.toRadians((cameraAngle + ty()))) * (cameraHeight - targetHeight);
        else
            d = 0.0;
        SmartDashboard.putNumber("Target Distance", d);
        return d;
    }

    public void setTargetHeight(TargetHeight height) {
        switch (height) {
        case BALL:
            targetHeight = 4.5;
            break;
        case HATCH_PANEL:
            targetHeight = 29.0;
        default:
            break;
        }
    }

    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void setLEDMode(ledMode mode) {
        int x = 0;
        switch (mode) {
        case OFF:
            x = 1;
            break;
        case BLINK:
            x = 2;
            break;
        case ON:
            x = 3;
            break;
        default:
            break;
        }
        table.getEntry("ledMode").setNumber(x);
    }

    public void setCamDMode(camMode mode) {
        int x = 0;
        switch (mode) {
        case DRIVERCAMERA:
            x = 1;
        default:
            break;
        }
        table.getEntry("camMode").setNumber(x);
    }

    public void setStreamMode(streamMode mode) {
        int x = 0;
        switch (mode) {
        case PiPMAIN:
            x = 1;
            break;
        case PiPSECONDARY:
            x = 2;
            break;
        default:
            break;
        }
        table.getEntry("stream").setNumber(x);
    }

    public double tx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double ty() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public double ta() {
        return table.getEntry("tx").getDouble(0.0);
    }
}