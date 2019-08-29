package frc.robot.navigation;

public class Point {
	private double x;
	private double y;
	private double targetV;
	private double dist_Along_Path;
	private double curvature;

	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Point(double x, double y, double tarV) {
		this.x = x;
		this.y = y;
		this.targetV = tarV;
	}

	/**
	 * @return the curvature
	 */
	public double getCurvature() {
		return curvature;
	}

	/**
	 * @param curvature the curvature to set
	 */
	public void setCurvature(double curvature) {
		this.curvature = curvature;
	}

	public void setDist_Along_Path(double d) {
		this.dist_Along_Path = d;
	}

	public double getDist_Along_Path() {
		return this.dist_Along_Path;
	}

	public void setVel(double v) {
		targetV = v;
	}

	public double getVel() {
		return targetV;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double angleBetween(Point p) {
		return Math.asin(y - p.getY() / x - p.getX());
	}

	public double distFrom(Point p) {
		return Math.sqrt(
				Math.abs(p.getX() - x) * Math.abs(p.getX() - x) + Math.abs(p.getY() - y) * Math.abs(p.getY() - y));
	}

	public String toString() {
		return "(" + x + ", " + y + ")";
	}

	public static double curvature(double lookAheadDistance, Point robotPosition, double heading,
			Point lookAheadPoint) {
		double robotX = robotPosition.getX();
		double robotY = robotPosition.getY();

		double a = -Math.tan(heading);
		double b = 1;
		double c = Math.tan(heading) * robotX - robotY;
		double side = Math.signum(Math.sin(heading) * (lookAheadPoint.getX() - robotX)
				- Math.cos(heading) * (lookAheadPoint.getY() - robotY));
		double x = Math.abs(a * lookAheadPoint.getX() + b * lookAheadPoint.getY() + c)
				/ Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
		double curvature = (2 * x) / (Math.pow(lookAheadDistance, 2));
		double sCurvature = side * curvature;

		if (Double.isNaN(sCurvature)) {
			return 0.001;
		}
		return sCurvature;
	}
}
