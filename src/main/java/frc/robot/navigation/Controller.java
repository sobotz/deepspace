package frc.robot.navigation;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Controller {
	
	Path genPath;
	
	
	private double time;			//time for time difference in rate limiter
	private double output;			//rate limiter output
	private Timer t = new Timer();	//timer for rate limiter
	private double maxRate;			//maximum rate of acceleration - rate limiter
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE LOCATION AND LOOK AHEAD VARAIBLES                     */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private Gyro g;							//gyroscope for angle
	private Point lPoint = new Point(0,0);	//look ahead point
	private double lDistance;				//look ahead distance
	private Point currentPosition;
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE RATE LIMITER VARIABLES		                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	

	private double xLocation;
	private double yLocation;
	private double distance;
	private Encoder lEncoder = new Encoder(0,1),
			rEncoder = new Encoder(2,3);	//encoders for getting location.
	
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE CONTROL LOOP VARIABLES		                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private double V;				//target robot velocity
	private double L;				//target Left wheels speed
	private double R;				//target right wheels speed
	private double C;				//curvature of arc
	private double T = 26.296875;	//track width
	private double tAccel;			//target acceleration
	
	private double kA = 0.00;//2;	//acceleration constant
	private double kP = 0.0;//1;	//proportional feedback constant
	private double kV = 3.3;		//velocity constant
	
	private double ffL;				//Left feed forward term
	private double fbL;				//Left feedback term
	private double ffR;				//Right feed forward term
	private double fbR;				//Right feedback term
	
	
	
	
	
	public Controller(Point[] path, double weight_smooth, double tol) {
		
		double a = 1 - weight_smooth;
		
		Path p = new Path(path);
		
		int[] numPoints = p.numPointForArray(6);
		
		this.genPath = new Path(p.generatePath(numPoints));
		
		genPath = genPath.smoother( a, weight_smooth, tol);
		genPath.setTarVel();
	}
	
	
	
	
	public void controlLoop() {
		
		distance = Math.abs((6*3.14)*(rEncoder.get() + lEncoder.get()/2)/360);
		xLocation = distance * Math.cos(g.getAngle());
		yLocation = distance * Math.sin(g.getAngle());
		currentPosition = new Point(xLocation, yLocation);
		
		while(genPath.size() > 1) {
			
			lPoint = Path.findLookAheadPoint(genPath, lDistance, currentPosition, lPoint);

			C = Point.curvature(lDistance, currentPosition, g.getAngle(), lPoint);
			
			genPath = Path.copyPath(genPath.closestPoint(currentPosition));
			
			V = rateLimiter(genPath.get(0).getVel());
			
			tAccel = (T * V) / 2;
			
			L = (V * (2 + (C * T))) / 2;
			R = (V * (2 - (C * T))) / 2;
			
			ffL = kV * L + kA * tAccel;
			ffR = kV * R + kA * tAccel;
			fbL = kP * (L - getSpeed());
			fbR = kP * (R - getSpeed());
			
			//Code to give each side of the drive train is the (FF + FB) for each side independently.
			
		}
		
	}
	
	public double getSpeed() {
		return 99;
	}

	public double rateLimiter(double input) {
		double deltaT = t.get() - this.time;
		this.time = t.get();
		double maxChange = deltaT * maxRate;
		output += Path.constrain(input - output, -maxChange, maxChange);
		return output;
	}
}