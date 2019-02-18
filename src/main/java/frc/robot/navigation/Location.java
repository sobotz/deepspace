package frc.robot.navigation;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Location {
	
	private Point currentPosition;
	private Encoder leftEncoder;
	private Encoder rightEncoder;
	private double xLocation;
	private double yLocation;
	private double distance;
	private Gyro gyro;
	
	public Location(Encoder left, Encoder right, Gyro a) {
		leftEncoder = left;
		rightEncoder = right;
		gyro = a;
		
		distance = Math.abs((6 * 3.14) * (rightEncoder.get() + leftEncoder.get() / 2) / 360);
		xLocation = distance * Math.cos(gyro.getAngle());
		yLocation = distance * Math.sin(gyro.getAngle());
		currentPosition = new Point(xLocation, yLocation);
	}

	public Point getCurrentPosition() {
		return currentPosition;
	}
	
	public void setCurrentPosition(Point currentPosition) {
		this.currentPosition = currentPosition;
	}

	/*
	public void move(double changeLeft, double changeRight, double angle) {
		double distance = (changeLeft + changeRight)/2.0;
		
		double newX = distance * Math.cos(Math.toRadians(angle));
		double newY = distance * Math.sin(Math.toRadians(angle));
		currentPosition.moveX(newX - currentPosition.getX());
		currentPosition.moveY(newY - currentPosition.getY());
	}
	*/
}