package frc.robot.navigation;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Vector extends Vector2d {
	
	public Vector(Point p1, Point p2) {
		super(p2.getX() - p1.getX(), p2.getY() - p1.getY());	
	}
	
}