package frc.robot.navigation;

import java.util.HashMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Controller {
    private Path genPath;
    private int lookAheadPointIndex; // index of the look ahead point
    private Point lookAheadPoint;
    private double lDistance = 6; // look ahead distance
    private Point robotPosition = new Point(1, 1);
    private Point closestPoint = null; // closest point
    private double xLocation;
    private double yLocation;
    private double distance;
    private double targetVelocity; // target robot velocity
    private double targetLeft;
    private double targetRight;
    private double curvature; // curvature of arc
    private double trackWidth = 26.296875; // track width
    public boolean isFinished = false;

    public Controller(Point[] points, double weight_smooth, double tol) {
        double a = 1 - weight_smooth;
        Path path = new Path(points);
        this.genPath = new Path(path.generatePath(path.numPointForArray(6)));
        this.genPath = this.genPath.smoother(a, weight_smooth, tol);
        genPath.calculatePointMetrix();
        genPath.adjustTargetVelocityForDeceleration();
        this.lookAheadPointIndex = 0;
        this.lookAheadPoint = new Point(genPath.get(lookAheadPointIndex).getX(),
                genPath.get(lookAheadPointIndex).getY());

        for (Point x : genPath) {
            System.out.println(x);
        }
    }

    public boolean controlLoop(double lPosition, double rPosition, double heading) {
        this.distance = Math.abs((6 * 3.14) * (rPosition + lPosition / 2) / 360);
        this.xLocation += this.distance * Math.cos(heading);
        this.yLocation += this.distance * Math.sin(heading);
        robotPosition = new Point(this.xLocation, this.yLocation);
        this.lookAheadPoint = genPath
                .get(genPath.findLookAheadIndex(this.lDistance, this.robotPosition, this.lookAheadPointIndex));
        this.curvature = Point.curvature(this.lDistance, robotPosition, heading, this.lookAheadPoint);
        this.closestPoint = genPath.get(genPath.closestPoint(robotPosition, 0));
        this.targetVelocity = genPath.rateLimiter(this.closestPoint.getVel());
        this.targetLeft = (this.targetVelocity * (2 + (this.curvature * this.trackWidth))) / 2;
        this.targetRight = (this.targetVelocity * (2 - (this.curvature * this.trackWidth))) / 2;

        Robot.m_drivesubsystem.PurePursuit(targetLeft, targetRight);

        if (genPath.indexOf(closestPoint) == (genPath.size() - 1)) {
            this.isFinished = true;
        }
        return this.isFinished;
    }
}
