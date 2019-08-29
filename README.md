# Destination: Deep Space
This is the code walkthrough for the 2019 FRC season. This year, some of our achievements include a pure pursuit algorithm, vision processing, and a command-based code base.

## Code Structure
This year, we've switched to a command-based system, which means that we implement tasks using commands derived from subsystems. Each subsystem corresponds to a specific component of the robot, such as `DriveSubsystem` to the drive train. From there, we are able to control motors and sensors to complete tasks. For example, `DriveCommand` uses the motors instantiated in `DriveSubsystem` to drive a certain distance.

For those looking in our code, the `autonomous` folder houses all of our autonomous paths for competition. The `navigation` folder contains the pure pursuit algorithm, and is implemented inside of the autonomous paths. The `subsystems` folder includes all of the components of the robot, and the `commands` folder implements the components instantiated in `subsystems`.

## Pure Pursuit
The pure pursuit algorithm enables us to drive in smooth curves during the autonomous period. Using calculus and a series of classes, we can map points in between points submitted to the function. 

```
this.distance = Math.abs((6*3.14)*(rPosition + lPosition/2)/360);
this.xLocation += this.distance * Math.cos(heading);
this.yLocation += this.distance * Math.sin(heading);
robotPosition = new Point(this.xLocation, this.yLocation);
```

This section of the code calculates the current position of the robot using cosine, sine, and the distance travels by the encoders from the last point. Knowing the robot's location on the field allows us to calculate the next step and ensure that we can make it to the final point of the algorithm.

```
public double rateLimiter(double targetInput) {
  double lastOutput = this.targetOutput;
  return this.targetOutput += constrain((targetInput - lastOutput), -MAXCHANGE, MAXCHANGE);
}
```

The rate limiter function limits the rate of acceleration of the robot to prevent it from crashing and to keep the motors from accelerating too fast. 

## Vision Processing
Our vision processing utilizes a Limelight camera, which helps us identify retroreflective tape on the field, and also enables us to align the robot with the white tape lines in front of the elements.

```
public boolean hasTarget() {
  if (table.getEntry("tv").getDouble(0) > 0.0) {
    return true;
  } else {
    return false;
  }
}
```

This snippet alerts returns `true` if the camera was able to identify a vision target, and `false` if there is no vision target in the field. When `true` is returned, the driver knows that they can deploy a routine from the button box to deposit an element or align with tape.

```
public double getTargetDistance() {
  double d = 0.0;
  if (hasTarget())
    d = Math.tan(Math.toRadians((_cameraAngle+ty())))*(_cameraHeight-_targetHeight);
  else
    d = 0.0;
  SmartDashboard.putNumber("Target Distance", d);
  return d;
}
```

The `getTargetDistance` function returns the distance from the the robot to the target that was attained using the camera. This value can be returned to other functions to tell the motor controllers how far they need to drive.

## Other Snippets
Autonomous paths are an integral part of our strategy. With our paths, we are able to place hatch panels during sandstorm. This way, we eliminate the human error that would occur with the use of a camera.

```
public PathL2C4(boolean type) {
  isPurePursuit = type;
  if (isPurePursuit) {
    Point[] path = {new Point(1,1,0), new Point(1,157.25)};
    addSequential(new PurePursuitCommand(path));
  } else {
    addSequential( new DriveToTargetCommand(156.25));
  }
}
```

Our simplest path, L2C4, travels straight from the middle section of Level 1 of the HAB to the 4th bay on the cargo ship. A boolean is passed through the function to determine whether to run pure pursuit or the regular autonomous. If pure pursuit is selected, the code runs the points inputted and travels between them. If regular is selected, the robot uses its encoders to drive 165 inches forwards.

```
protected void execute() {
  Robot.m_drivesubsystem.manualDrive(Robot.m_oi.m_driverjoystick.getRawAxis(1), Robot.m_oi.m_driverjoystick.getRawAxis(0));
}
```

This snippet is what runs our drivetrain during matches. Using input from a joystick, the `manualDrive` method sets the speed of each speed controller set. Differing speeds in each set of wheels allows us to drive like a tank, and turn by running one set of wheels faster than the other.

## Resources
We used [this](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) paper to help us write our pure pursuit algorithm.
