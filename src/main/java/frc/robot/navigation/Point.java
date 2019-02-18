package frc.robot.navigation;

public class Point {
	private double x;
	private double y;
	private double targetV;
	private double maxV;
	private double fIndex;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Point(double x, double y, double tarV) {
		this.x = x;
		this.y = y;
		this.targetV = tarV;
	}
	
	public void setMaxV(double v) {
		maxV = v;
	}
	
	public double getMaxV() {
		return maxV;
	}

	public void setVel(double v){
		targetV = v;
	}

	public double getVel(){
		return targetV;
	}

	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}

	public double fIndex() {
		return fIndex;
	}

	public void setIndex(double i) {
		fIndex = i;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public void setY(double y) {
		this.y = y;
	}
	
	public double angleBetween(Point p) {
		return Math.asin(y-p.getY()/x-p.getX());
	}

	public double distFrom(Point p) {
		return Math.sqrt(Math.abs(p.getX()-x)*Math.abs(p.getX()-x) + Math.abs(p.getY()-y)*Math.abs(p.getY()-y));
	}
	
	public String toString() {
		return "(" + x + ", " + y + ")";
	}

	public static double curvature(double L, Point cPosition, double rAngle, Point lPoint) {
		
		//variable instantiation
		double localX = cPosition.getX();
		double localY = cPosition.getY();
		
		double a = -Math.tan(rAngle);
		double b = 1;
		double c = Math.tan(rAngle) * localX;
		
		//calculations
		double side = Math.signum(Math.sin(rAngle) * (lPoint.getX() - localX) 
				- Math.cos(rAngle) * (lPoint.getY() - localY));
		double x = Math.abs(a * lPoint.getX() + b * lPoint.getY() + c)
				/ Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
		double curvature = (2*x)/(Math.pow(L, 2));
		double sCurvature = side * curvature;
		
		return sCurvature;
	}
}