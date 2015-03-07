package ui;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Arc2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import optimalControl.Homogeneous;
import optimalControl.Trajectory;
import optimalControl.Transformation;

public class TrajectoryShape {
	private static final AffineTransform scale = AffineTransform.getScaleInstance(Viewer.SCALE, Viewer.SCALE);
	private ArrayList<Shape> trace;

	public TrajectoryShape(Transformation Ts, Trajectory trajectory) {
		trace = getTrace(Ts, trajectory);
	}
	
	/**
	 * @param g the graphics to be drawn
	 * @param time the final time
	 */
	public void draw(Graphics2D g) {
		g.setStroke(new BasicStroke(3.0f));
		drawTrace(g);
	}
	
	/**
	 * Draw the trajectory of the reference point
	 * @param g the graphics to be drawn
	 */
	private void drawTrace(Graphics2D g) {		
		for (Shape s : trace) {
			g.draw(scale.createTransformedShape(s));
		}
	}
	
	/**
	 * Compute the path of the center along the trajectory
	 * @param T transformation for the configuration
	 * @param traj trajectory
	 * @return
	 */
	public static ArrayList<Shape> getTrace(Transformation T, Trajectory traj) {
		ArrayList<Point2D> endPoints = endPoints(T, traj);
		ArrayList<Point2D> rcs = rotationCenters(T, traj);
		ArrayList<Shape> result = new ArrayList<>();
		for (int i = 0; i < traj.size(); ++i) {
			if (traj.getControl(i).isRotation()) {
				Point2D rc = rcs.get(i);
				Point2D begin = endPoints.get(i);
				result.add(computeArc(rc, begin, traj.getControl(i).getOmega(), traj.getDuration(i)));
			} else {
				Line2D L = new Line2D.Double(endPoints.get(i), endPoints.get(i+1));
				result.add(L);
			}
		}
		return result;
	}
	
	/**
	 * @param T transformation for the initial configuration
	 * @param traj trajectory
	 * @return
	 */
	public static ArrayList<Point2D> endPoints(Transformation T, Trajectory traj) {
		ArrayList<Point2D> result = new ArrayList<>();
		result.add(T.toPoint());
		for (int i = 0; i < traj.size(); ++i) {
			T = T.move(traj.getControl(i), traj.getDuration(i));
			result.add(T.toPoint());
		}
		return result;
	}
	
	/**
	 * 
	 * @param T transformation for the configuration
	 * @param traj trajectory
	 * @return
	 */
	public static ArrayList<Point2D> rotationCenters(Transformation T, Trajectory traj) {
		ArrayList<Point2D> result = new ArrayList<>();
		for (int i = 0; i < traj.size(); ++i) {
			if (traj.getControl(i).isRotation()) {
				result.add(T.transform(new Homogeneous(traj.getControl(i))).toPoint());
			} else 
				result.add(null);
			T = T.move(traj.getControl(i), traj.getDuration(i));
		}
		return result;
	}
	
	/**
	 * Compute the arc from rotation center and the begin point, angular velocity, and duration
	 * @param rc Rotation center
	 * @param begin Begin point
	 * @param omega Angular velocity
	 * @param duration duration of the control
	 * @return
	 */
	public static Arc2D computeArc(Point2D rc, Point2D begin, double omega, double duration) {
		double radius = rc.distance(begin);
		double startAngle = Math.toDegrees(Math.atan2(begin.getY() - rc.getY(), begin.getX() - rc.getX()));
		double durationInDegree = Math.toDegrees(duration);
		if (durationInDegree < 0.0) {
			durationInDegree += 360.0;
		}
		durationInDegree *= omega;
		Arc2D result = new Arc2D.Double();
		result.setArcByCenter(rc.getX(), rc.getY(), radius, -startAngle, -durationInDegree, Arc2D.OPEN);
		return result;
	}
}
