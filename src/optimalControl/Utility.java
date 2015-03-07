package optimalControl;
import java.awt.Shape;
import java.awt.geom.Arc2D;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;


public interface Utility {
	public static final double EPSILON = 0.0000001;  // Error tolerance
	
	/**
	 * Test whether a and b are approximate
	 * @param a
	 * @param b
	 * @return
	 */
	public static boolean absEqual(double a, double b) {
		double diff = Math.abs(a - b);
		if (diff < EPSILON) 
			return true;
		return diff / (Math.abs(a) + Math.abs(b)) < EPSILON;
	}
	
	/**
	 * Test whether x is arppxoaimtely zero
	 * @param x
	 * @return
	 */
	public static boolean isZero(double x) {
		return absEqual(x, 0.0);
	}
	
	/**
	 * Compute the time of rotating angle a to b with angular velocity omega
	 * @param a initial angle
	 * @param b final angle
	 * @param omega angular velocity
	 * @return time
	 */
	public static double timeToAngle(double a, double b, double omega) {
		return timeOfRotateTheta(b - a, omega);
	}
	
	/**
	 * Compute the time of rotating angle theta with angular velocity omega
	 * @param theta
	 * @param omega
	 * @return
	 */
	public static double timeOfRotateTheta(double theta, double omega) {
		double diff = normalize(theta);
		if (Utility.isZero(diff) || Utility.absEqual(diff, 2.0 * Math.PI))
			return 0.0;
		if (diff * omega > 0)
			return diff / omega;
		return (diff + (omega > 0 ? 2 : -2) * Math.PI) / omega;
	}
	
	/**
	 * Normalize an angle x to an angle in [0 .. 2pi)
	 * @param x
	 * @return
	 */
	public static double normalize(double x) {
		double temp = x % (Math.PI * 2);
	    return temp >= 0.0 ? temp : temp + Math.PI * 2;
	}
	
	/**
	 * Test whether two angles are the same
	 * @param a
	 * @param b
	 * @return
	 */
	public static boolean angleEqual(double a, double b) {
		double na = normalize(a), nb = normalize(b);
		if (Utility.absEqual(na, nb))
	        return true;
	    return (Utility.isZero(na) || Utility.absEqual(na, Math.PI * 2))
	        && (Utility.isZero(nb) || Utility.absEqual(nb, Math.PI * 2));
	}
	
	/**
	 * Compute the distance from p to the origin
	 * @param p
	 * @return the distance from p to the origin
	 */
	public static double distanceToOrigin(Point2D p) {
		final Point2D origin = new Point2D.Double();
		return origin.distance(p);
	}
	
	/**
	 * For a transformation T, apply rotation u1 until the vector from
     * rotation center of u1 to rotation center of u2 is the same as direction
	 * @param T
	 * @param u1
	 * @param u2
	 * @param direction
	 * @return
	 */
	public static double timeToLine(Transformation T, Control u1, Control u2, double direction) {
	    Point2D c1 = T.transform(new Homogeneous(u1)).toPoint();
	    Point2D c2 = T.transform(new Homogeneous(u2)).toPoint();
	    double theta = Math.atan2(c2.getY() - c1.getY(), c2.getX() - c1.getX());
	    return Utility.timeToAngle(theta, direction, u1.getOmega());
	}
	
	/**
	 * Compute the duration of u such that the rotation centers of previous, u, and next are aligned
	 * @param previous
	 * @param u
	 * @param next
	 * @return duration
	 */
	public static double timeToLine(Control previous, Control u, Control next) {
		Point2D cp = previous.rotationCenter();
		Point2D c = u.rotationCenter();
		Point2D cn = next.rotationCenter();
		double theta0 = Math.atan2(cn.getY() - c.getY(), cn.getX() - c.getX());
		double theta1 = Math.atan2(c.getY() - cp.getY(), c.getX() - cp.getX());
		return timeToAngle(theta0, theta1, u.getOmega());
	}
	
	/**
	 * @param T: Transformation of the initial configuration
	 * @param traj: Trajectory want to integrate
	 * @param time: End time
	 * @return
	 */
	public static Point2D rotationCenter(Transformation T, Trajectory traj, double time) {
		double sum = 0.0;
		Control u = null;
		for (int i = 0; i < traj.size(); ++i) {
			if (time >= sum && time < sum + traj.getDuration(i)) {
				T = T.move(traj.getControl(i), time - sum);
				u = traj.getControl(i);
				break;
			}
			T = T.move(traj.getControl(i), traj.getDuration(i));
			sum += traj.getDuration(i);
		}
		if (u == null || !u.isRotation())
			return null;
	    return T.transform(new Homogeneous(u)).toPoint();
	}
	
	/**
	 * Create a vector from origin to destination
	 * @param origin
	 * @param destination
	 * @return
	 */
	public static Point2D vector(Point2D origin, Point2D destination) {
		return new Point2D.Double(destination.getX() - origin.getX(), destination.getY() - origin.getY());
	}
	
	/**
	 * Compute the angle from vector v1 to v2
	 * if direction is true, then the angle is counterclockwise
	 * else the angle is clockwise
	 * @param v1 a point representing a vector
	 * @param v2 a point representing a vector
	 * @param direction
	 * @return the angle from v1 to v2 with respect to the direction
	 */
	public static double angle(Point2D v1, Point2D v2, boolean direction) {
		double dot = v1.getX() * v2.getY() - v1.getY() * v2.getX();
	    if (direction == false)
	        dot = -dot;
	    return Math.atan2(dot, v1.getX() * v2.getX() + v1.getY() * v2.getY());
	}
	
	/**
	 * Let c, c1, and c2 be the rotation centers of u, u1, and u2 respectively
	 * Compute the angle from the vector (c, c1) to (c, c2) 
	 * @param u1
	 * @param u2
	 * @param u3
	 * @return
	 */
	public static double angle(Control u, Control u1, Control u2) {
		Point2D c = u.rotationCenter();
		Point2D c1 = u1.rotationCenter();
		Point2D c2 = u2.rotationCenter();
		
		return angle(vector(c, c1), vector(c, c2), true);
	}
}
