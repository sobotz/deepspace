package frc.robot.navigation;

import frc.robot.Robot;

public class Controller {
    
    
    /*----------------------------------------------------------------------------*/
    /*                                                                       	  */
    /* DECLARATION OF THE PATH OBJECT      	                	             	  */
    /*                                                                       	  */
    /*----------------------------------------------------------------------------*/

    private Path genPath;
    
    /*----------------------------------------------------------------------------*/
    /*                                                                       	  */
    /* DECLARATION OF THE LOOK AHEAD VARAIBLES                 	             	  */
    /*                                                                       	  */
    /*----------------------------------------------------------------------------*/
    

	private int lookAheadPointIndex = 0;  		//index of the look ahead point
	private Point lookAheadPoint = new Point(1,1);
    private double lDistance = 6;               	//look ahead distance
    private Point robotPosition = new Point(0,0); 
    private Point closestPoint = null;  		//closest point
    

    /*----------------------------------------------------------------------------*/
    /*                                                                       	  */
    /* DECLARATION OF THE RATE LIMITER VARIABLES   	                          	  */
    /*                                                                       	  */
    /*----------------------------------------------------------------------------*/
    

    private double xLocation;
    private double yLocation;
    private double distance;
    
    
    /*----------------------------------------------------------------------------*/
    /*                                                                         	  */
    /* DECLARATION OF THE CONTROL LOOP VARIABLES   	 	                          */
    /*                                                                       	  */
    /*----------------------------------------------------------------------------*/
    
	private double targetVelocity;               //target robot velocity
	private double targetLeft;
	private double targetRight;

	/*
    private double LeftOriginal;              //target Left wheels speed
    private double RightOriginal;              //target right wheels speed
    private double LeftFinal;              //target Left wheels speed
	private double RightFinal;              //target right wheels speed
	*/
    private double curvature;               //curvature of arc
	private double trackWidth = 26.296875;   //track width
	
    //private double targetAccelerationLeft;          //target acceleration
    //private double targetAccelerationRight;          //target acceleration
    
    //private double kA = 0.002;   //acceleration constant
    //private double kP = 0.01;    //proportional feedback constant
    //private double kV = 3.3;        //velocity constant
    
    //private double ffL;             //Left feed forward term
    //private double fbL;             //Left feedback term
    //private double ffR;             //Right feed forward term
    //private double fbR;             //Right feedback term
    //private Double Left;
	//private Double Right;
    
    public boolean isFinished = false;
    
    
    
    public Controller(Point[] points, double weight_smooth, double tol) {
        
        double a = 1 - weight_smooth;

        Path path = new Path(points);

        this.genPath = new Path(path.generatePath(path.numPointForArray(6)));

        this.genPath = this.genPath.smoother(a, weight_smooth, tol);

		genPath.calculatePointMetrix();
		genPath.adjustTargetVelocityForDeceleration();
        
    }
    
    
    
	public boolean controlLoop(double leftChange, double rightChange, double heading) {
        
        this.distance = (rightChange + leftChange) / 2;

        /*
        heading += 90;
        if(heading > 360) {
            heading -= 360;
        }
        */

        System.out.println("[Heading: " + heading + "]");

        this.xLocation += this.distance * Math.cos(Math.toRadians(heading));
        this.yLocation += this.distance * Math.sin(Math.toRadians(heading));
        this.robotPosition = new Point(this.xLocation, this.yLocation);

        System.out.println("[Robot Position: " + this.robotPosition.toString() + "]");
            
        int nextLookAheadIndex = genPath.findLookAheadIndex(this.lDistance, this.robotPosition, this.lookAheadPointIndex);
        
        if(this.lookAheadPointIndex != nextLookAheadIndex) {
            System.err.println("Look Ahead Point changed: " + this.lookAheadPointIndex + " to " 
                + nextLookAheadIndex);
        }
        
        this.lookAheadPointIndex = nextLookAheadIndex;
        this.lookAheadPoint = genPath.get(nextLookAheadIndex);
        
        //this.lookAheadPoint = genPath.get(genPath.findLookAheadIndex(this.lDistance, this.robotPosition, 
        //  this.lookAheadPointIndex));

        //System.out.println("[Look Ahead Point: " + this.lookAheadPoint.toString() + "]");

        this.curvature = Point.curvature(this.lDistance, this.robotPosition, heading, this.lookAheadPoint);

        System.out.println("[Curvature: " + this.curvature + "]");

        this.closestPoint = genPath.get(genPath.closestPoint(this.robotPosition, 0));
        
        System.out.println("[Closest Point: " + this.closestPoint.toString() + "]");
			
        this.targetVelocity = genPath.rateLimiter(this.closestPoint.getVel());
        
        System.out.println("[Target Velocity: " + this.targetVelocity + "]");
            
        this.targetLeft = (this.targetVelocity * (2 + (this.curvature * this.trackWidth))) / 2;
        this.targetRight = (this.targetVelocity * (2 - (this.curvature * this.trackWidth))) / 2;

        double talonLeftTarget = Robot.m_drivesubsystem.velocityToTalonVelocity(targetLeft);
        double talonRightTarget = Robot.m_drivesubsystem.velocityToTalonVelocity(targetRight);
        
        System.out.println("[Target Left Velocity: " + this.targetLeft + "]");
        System.out.println("[Target Left Velocity(Talon): " + talonLeftTarget + "]");
        System.out.println("[Target Right Velocity: " + this.targetRight + "]");
        System.out.println("[Target Right Velocity(Talon): " + talonRightTarget + "]");

        Robot.m_drivesubsystem.PurePursuit(targetLeft, targetRight);
			
		/*
        double dist = robotPosition.distFrom(this.lookAheadPoint);
        this.targetAccelerationLeft = ( ( (this.LeftFinal * this.LeftFinal) - (this.LeftOriginal * 
            this.LeftOriginal) ) / (2 * dist) );
        this.targetAccelerationRight = ( ( (this.RightFinal * this.RightFinal) - (this.RightOriginal * 
            this.RightOriginal) ) / (2 * dist) );
            
        this.ffL = this.kV * this.LeftFinal + this.kA * this.targetAccelerationLeft;
        this.ffR = this.kV * this.RightFinal + this.kA * this.targetAccelerationRight;
        this.fbL = this.kP * (this.LeftFinal - lSpeed);
        this.fbR = this.kP * (this.RightFinal - rSpeed);
            
        this.Left = (this.ffL + this.fbL);
        this.Right = (this.ffR + this.fbR);
        this.LeftOriginal = this.RightFinal;
        this.RightOriginal = this.RightFinal;
            
        this.wV.put("Left", Left);
        this.wV.put("Right", Right);
            
		return this.wV;
        */
        
        if(genPath.indexOf(closestPoint) == (genPath.size() - 1)) {
            this.isFinished = true;
        }
        
        System.out.println("[isFinished: " + this.isFinished + "]");
        
        return this.isFinished;
    }
	
	public Path getPath() {
		return this.genPath;
	}
}