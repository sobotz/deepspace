package frc.robot.navigation;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Controller {
	
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE PATH OBJECT			          			   	          */
	/*																			  */
	/*----------------------------------------------------------------------------*/

	Path genPath;
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE LOOK AHEAD VARAIBLES          			   	          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	

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
	private double time;			//time for time difference in rate limiter
	private double output;			//rate limiter output
	private Timer t = new Timer();	//timer for rate limiter
	private double maxRate;			//maximum rate of acceleration - rate limiter
	
	
	/*----------------------------------------------------------------------------*/
	/*																			  */
	/* INSTANTIATION OF THE CONTROL LOOP VARIABLES		                          */
	/*																			  */
	/*----------------------------------------------------------------------------*/
	
	private double V;				//target robot velocity
	private double LO = 0;				//target Left wheels speed
	private double LF;				//target Left wheels speed
	private double RO = 0;				//target right wheels speed
	private double RF;				//target right wheels speed
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
	private Double Left;
	private Double Right;
	
	private HashMap<String, Double> wV = new HashMap<>();
	public boolean isFinished = false;
	
	
	
	public Controller(Point[] path, double weight_smooth, double tol) {
		
		double a = 1 - weight_smooth;
		
		Path p = new Path(path);
		
		int[] numPoints = p.numPointForArray(6);
		
		this.genPath = new Path(p.generatePath(numPoints));
		
		genPath = genPath.smoother( a, weight_smooth, tol);
		genPath.setTarVel();
	}
	
	
	
	public HashMap<String, Double> controlLoop(double lPosition, double rPosition, double heading, double lSpeed, double rSpeed) {
		
		distance = Math.abs((6*3.14)*(rPosition + lPosition/2)/360);
		xLocation = distance * Math.cos(heading);
		yLocation = distance * Math.sin(heading);
		currentPosition = new Point(xLocation, yLocation);

		while(genPath.size() > 1 || isFinished != false) {
			
			lPoint = Path.findLookAheadPoint(genPath, lDistance, currentPosition, lPoint);

			C = Point.curvature(lDistance, currentPosition, heading, lPoint);
			
			genPath.setTarVel();

			genPath = Path.copyPath(genPath.closestPoint(currentPosition));
			
			V = rateLimiter(genPath.get(0).getVel());
			
			LF = (V * (2 + (C * T))) / 2;
			RF = (V * (2 - (C * T))) / 2;
			
			double distance = currentPosition.distFrom(lPoint);

			tAccel = rateLimiter(((LF * LF) - (LO * LO)) / (2 * distance));
			
			ffL = kV * LF + kA * tAccel;
			ffR = kV * RF + kA * tAccel;
			fbL = kP * (LF - lSpeed);
			fbR = kP * (RF - rSpeed);
			
			Left = (ffL + fbL);
			Right = (ffR + fbR);

			LO = LF;
			RO = RF;
			
			wV.put("Left", Left);
			wV.put("Right", Right);
			
			return wV;
		}
		
		this.isFinished = true;
		return wV;
	}

	public double rateLimiter(double input) {
		double deltaT = t.get() - this.time;
		this.time = t.get();
		double maxChange = deltaT * maxRate;
		output += Path.constrain(input - output, -maxChange, maxChange);
		return output;
	}
}
