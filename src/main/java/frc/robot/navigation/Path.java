package frc.robot.navigation;
import java.util.ArrayList;
/**
 * @author ncabrera528@gmail.com(Nicholas Cabrera)
 */

import edu.wpi.first.wpilibj.drive.Vector2d;

@SuppressWarnings("serial")
public class Path extends ArrayList<Point>{

	/**
	The constructors for the Path class.
	@author ncabrera528@gmail.com(Nicholas Cabrera)
	**/
	private double maxVelocity;
	private final double DEFAULTMAX = 15;
	private final double DEFAULTACCELERATION = 7;
	private final double MAXCHANGE = DEFAULTACCELERATION * 0.02;
	private double CURVE_SENSITIVITY_CONSTANT = .1;
	private double fIndex = 0;
	private double targetOutput = 0;
	
	public Path() {
		super();
		this.maxVelocity = DEFAULTMAX;
	}

	public Path(Iterable<Point> path) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
		this.maxVelocity = DEFAULTMAX;
		
	}
	
	public Path(Point[] path) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
		this.maxVelocity = DEFAULTMAX;
	}

	public Path(Iterable<Point> path, double maxVelocity) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
		this.maxVelocity = maxVelocity;
	}
	
	public Path(Point[] path, double maxVelocity) {
		for(Point g: path)
			add(new Point(g.getX(), g.getY()));
		this.maxVelocity = maxVelocity;
	}
	
	/**
	This code will help with going from path to matrix, and back, as the code for 
	smoothing the path needs a matrix instead of the current ArrayList of Points
	
	@author ncabrera528@gmail.com(Nicholas Cabrera)
	**/

	/**
	What is happening in this code is instantiation, then a for loop where it loops 
	through each element of the ArrayList, and puts the X value for each point as 
	the first column, and the Y value for each point as the second column
	**/

	/**
	What is happening in this code is instantiation, then a for loop where it loops 
	through each row, and where the first column is the X value, and the second 
	column is the Y value, adds a new Point with those values of X and Y
	**/

	public static double[][] pathToMatrix(Path path){
		double[][] genPath = new double[path.size()][2];
		for(int i = 0; i < path.size(); i++){
			genPath[i][0] = path.get(i).getX();
			genPath[i][1] = path.get(i).getY();
		}
		return genPath;
	}
	
	public static Path matrixToPath(double[][] path){
		ArrayList<Point> temp = new ArrayList<Point>();
		for(int i = 0; i < path.length; i++)
			temp.add(new Point(path[i][0],path[i][1]));
		Path genPath = new Path(temp);
		return genPath;
	}

	/**
	The generatePath method uses the numPointForArray method to determine the amount
	of points should go into a line segment based on the distance between each point
	
	@author ncabrera528@gmail.com(Nicholas Cabrera)
	**/
	
	public int[] numPointForArray(double dist) {
		int[] numPoints = new int[size()-1];
		for(int i = 0; i < numPoints.length; i++)
			numPoints[i] = (int)((get(i).distFrom(get(i+1)))/dist)-1;
		return numPoints;
	}
	
	/**
	The generate path class is the class that injects points into the path. to do 
	this,it takes the x and y dimensions of the line segment, divides that number by 
	the number of points you want to be in that segment + 1(because the last point
	will always be the end of the line segment, to get four points into a line 
	segment you must divide it by 5) to get the distance each point will be away 
	from each other. Then, it adds the first point, injects the points, and at the 
	end of the algorithm, adds the last point.
	
	The function of the double for loop is this, the first for loop will loop through 
	each line segment, and the second for loop is used to inject the points onto the 
	path.

	@author ncabrera528@gmail.com(Nicholas Cabrera)
	**/
	
	public ArrayList<Point> generatePath(int[] numPoints){
		ArrayList<Point> genPath = new ArrayList<Point>();
		double dimensionX = 0; 
		double dimensionY = 0; 
		double distanceX = 0; 
		double distanceY = 0;
		Point temp = new Point(0,0);
		
		for(int i = 0; i <= size() - 2; i++)
		{
			dimensionX = get(i + 1).getX() - get(i).getX();
			dimensionY = get(i + 1).getY() - get(i).getY();
			
			distanceX = dimensionX / (numPoints[i] + 1);
			distanceY = dimensionY / (numPoints[i] + 1);
			
			temp = new Point(get(i).getX(), get(i).getY());
			genPath.add(new Point(temp.getX(), temp.getY()));

			for(int x = 0; x < numPoints[i]; x++)
			{
				temp.setX(temp.getX() + distanceX);
				temp.setY(temp.getY() + distanceY);
				genPath.add(new Point(temp.getX(), temp.getY()));
			}
		}
		genPath.add(new Point(get(size()-1).getX(), get(size()-1).getY()));
		return genPath;
	}

	/**
	This method calculates the curve.
	It takes a path, and parameters a, b, and tolerance. This algorithm is borrowed from Team 2168, and it is
	recommended that b be within .75 and .98, with a set to 1 - b, and tolerance = 0.001.
	**/

	public static double[][] smoother(double[][] path, double a, double b, 
			double tolerance) {
		double[][] genPath = new double[path.length][path[0].length];
		for(int r = 0; r < path.length; r++)
			for(int c = 0; c < path[0].length; c++)
				genPath[r][c] = path[r][c];
		double change = tolerance;
		
		while(change >= tolerance){
			change = 0.0;
			for(int row = 1; row < path.length - 1; row++){
				for(int col = 0; col < path[row].length; col++){
					double temp = genPath[row][col];
					genPath[row][col] += a * (path[row][col] - genPath[row][col]) 
							+ b * (genPath[row - 1][col] + genPath[row + 1][col] 
									- (2.0 * genPath[row][col]));
					change += Math.abs(temp - genPath[row][col]);
				}
			}
		}
		return genPath;
	}

	/**
	The modified smoother class is an instance class that was created specifically so that instead of having
	to transition all the Path objects to double[][] in the main method, we could call this method to do it for us.
	
	@author Peter Wu
	**/

	public Path smoother(double a, double b, double tolerance){
		return new Path(matrixToPath(smoother(pathToMatrix(this), a, b,tolerance)));
	}

	/**
	The formula here uses a systems of equations format to find the radius of the 
	circle, then to get the curvature of the circle. The purpose of it is to find 
	the curvature of the turn the robot wants to take, so that it can modulate its 
	speed based on the curvature of the turn. 
	Usually, the parameters should be the point you want to turn at, and the points 
	on either side of it, where Q is on the leftmost of the turn, R is the rightmost,
	and P is the desired point of curvature. 
	Some notes, if the result is NaN, that means the curvature is zero and the 
	radius is infinite, so therefore the path is a straight. Also, if x1 is equal 
	to x2, then you get a divide by zero error. To fix this, add a small value to x1,
	such as 0.001, and the issue will be fixed with minimal error.
	@author ncabrera528@gmail.com(Nicholas Cabrera)
	**/
	
	public static double curvatureOfArc(Point P, Point Q, Point R){
		double xOne = P.getX();
		double xTwo = Q.getX();
		double xThree = R.getX();
		double yOne = P.getY();
		double yTwo = Q.getY();
		double yThree = R.getY();

		if(xOne == xTwo) {
			xOne += 0.001;
		}

		double kOne = 0.5 * ((Math.pow(xOne, 2) + Math.pow(yOne, 2) - Math.pow(xTwo, 2) 
				- Math.pow(yTwo, 2)) / (xOne-xTwo));

		double kTwo = (yOne -yTwo) / (xOne-xTwo);

		double b = 0.5 * (Math.pow(xTwo, 2) - 2 * xTwo * kOne + Math.pow(yTwo, 2)
				- Math.pow(xThree, 2) + 2  * xThree * kOne - Math.pow(yThree, 2))
				/ (xThree * kTwo - yThree + yTwo - xTwo * kTwo);

		double a = kOne - kTwo * b;

		double r = Math.sqrt(Math.pow((xOne - a), 2)  + Math.pow((yOne - b), 2));

		double curvature = 1 / r;

		if(Double.isNaN(curvature)) {		//if line is straight, then curve is flat
			return 0.000001;
		}

		return curvature;
	}

	/**
	 * @author ncabrera528@gmail.com(Nicholas Cabrera)
	**/

	public void adjustTargetVelocityForDeceleration() {

		/*
		Note, you are iterating backwards so previous would technically have a greater index
		*/

		Point current = null;
		Point previous = null;
		double distance = 0;
		double oldVelocity = 0; 

		for(int i = size() - 1; i >= 0; i--) {

			if(i == size() - 1) {
				get(i).setVel(0);
			} else {
				current = get(i);
				previous = get(i + 1);
				distance = previous.getDist_Along_Path() - current.getDist_Along_Path();

				oldVelocity = current.getVel();

				current.setVel(Math.min(oldVelocity, Math.sqrt(Math.pow(previous.getVel(), 2) 
					+ 2 * this.DEFAULTACCELERATION * distance)));
			}

		}

	}

	/**
	 * @author ncabrera528@gmail.com(Nicholas Cabrera)
	**/
	
	public void calculatePointMetrix(){		//Set distance at point and curvature
		Point previous = null;
		Point current = null;
		Point next = null;

		for(int i = 0; i < size(); i ++) {

			previous = current;
			current = get(i);

			if (i == 0){
				current.setDist_Along_Path(0);
				current.setCurvature(0);
			} else {

				current.setDist_Along_Path((previous.getDist_Along_Path() + current.distFrom(previous)));
	
				if(i == size() - 1) {
					current.setCurvature(0);
				}
				else {
					next = get(i + 1);
					double curvature = curvatureOfArc(previous, current, next);
					current.setCurvature(curvature);
				}
			}

			current.setVel(Math.min(this.maxVelocity, (this.CURVE_SENSITIVITY_CONSTANT / current.getCurvature())));

		}



	}
	
	/**
	The constrain method takes a value as well as a minimum and a maximum and 
	constrains the value to be within the range.
	@author ncabrera528@gmail.com(Nicholas Cabrera)
	**/
	
	public static double constrain(double num, double min, double max) {
		if(num <= max && num >= min) {
			return num;
		} else if(num > max) {
			return max;
		} else if(num < min) {
			return min;
		}
		return 0;
	}

    public double rateLimiter(double targetInput) {
		double lastOutput = this.targetOutput;
		return this.targetOutput += constrain((targetInput - lastOutput), -MAXCHANGE, MAXCHANGE);
	}
	
	/**
	 * @author ncabrera528@gmail.com(Nicholas Cabrera)
	 * @param robotPosition
	 * @param previousIndex
	 * @return closestPoint
	 */
	
	public int closestPoint(Point robotPosition, int previousIndex) {
		
		int closestIndex = previousIndex;

		if(robotPosition.distFrom(get(previousIndex)) < 0.5) {
			previousIndex += 1;
		}

		for(int i = previousIndex; i < size(); i++) {
			if(robotPosition.distFrom(get(i)) <= robotPosition.distFrom(get(previousIndex))) {
				closestIndex = i;
			}
		}
		
		return closestIndex;
	}
	
	/**
	The lookAhead method uses quadratic equation to find intersect points, then 
	passes the x(AKA the roots) values on to the lookAheadPoint class.
	**/
	
	/**
	 * @author ncabrera528@gmail.com(Nicholas Cabrera)
	 * @param E
	 * @param L
	 * @param C
	 * @param r
	 * @return 
	 */
	
	public static double lookAhead(Point E,  Point L, Point C, double r) {
		Vector2d d = new Vector(E,L);
		Vector2d f = new Vector(C,E);
		
		double a = d.dot(d);
		double b = 2 * f.dot(d);
		double c = f.dot(f) - r*r ;
		
		double discriminant = ((Math.pow(b,2)) - (4*a*c));
		
		if( discriminant < 0 ) {
			return -1;
		}
		
		discriminant = (double)Math.sqrt(discriminant);
		
		double x1 = (-b - discriminant) / (2*a);
		double x2 = (-b + discriminant) / (2*a);
		

		if(x1 >= 0 && x1 <= 1) {
		    return x1;
		}
		
		if(x2 >= 0 && x2 <= 1) {
			return x2;
		}
		
		return -1;
	}

	/**
	 * @author Nicholas Cabrera
	 * @param lookAheadDistance
	 * @param robotPosition
	 * @param lastLookAheadPointIndex
	 * @return
	 */

	public int findLookAheadIndex(double lookAheadDistance, Point robotPosition, int lastLookAheadPointIndex) {

		double fractional = 0;
		Point current = null;
		Point next = null;
		int i = 0;

		for(i = lastLookAheadPointIndex; i < size() - 1; i++) {
			current = get(i);
			next = get(i + 1);
	
			fractional = lookAhead(current, next, robotPosition, lookAheadDistance);

			if(fractional >= 0 && fractional <= 1) {
				if((i + fractional) > this.fIndex) {
					fIndex = (i + fractional);
					return (i + 1);
				}
			}
		}

		return lastLookAheadPointIndex;
	}
}