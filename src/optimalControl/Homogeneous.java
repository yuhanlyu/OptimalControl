package optimalControl;
import java.awt.geom.Point2D;

/**
 * A class representing a homogeneous configuration
 * @author yu-hanlyu
 *
 */
public class Homogeneous {
	private double x; // x-coordinate
	private double y; // y-coordinate
	private double omega; // orientation
	
	/**
	 * @param u Represent a control in homegeneous coordinate
	 */
	public Homogeneous(Control u) {
		this.x = -u.getVy();
		this.y = u.getVx();
		this.omega = u.getOmega();
	}
	
	/**
	 * @param x x-coordinate
	 * @param y y-coordinate
	 * @param omega angular velocity
	 */
	public Homogeneous(double x, double y, double omega) {
		this.x = x;
		this.y = y;
		this.omega = omega;
	}
	
	/**
	 * @return a point represented by the homogeneous coordinate
	 */
	public Point2D toPoint() {
		return new Point2D.Double(x / omega, y / omega);
	}
	
	/**
	 * @return x-coordinate without dividing by omega
	 */
	public double getX() {
		return x;
	}
	
	/**
	 * @return y-coordinate without dividing by omega
	 */
	public double getY() {
		return y;
	}
	
	/**
	 * @return omega
	 */
	public double getOmega() {
		return omega;
	}
	
	/**
	 * Test whether two homogeneous points are the same
	 */
	public boolean samePoint(Homogeneous Q) {
		double f = 0.0;
	    if (!Utility.isZero(Q.getX())) {
	        f = getX() / Q.getX();
	    } else if (!Utility.isZero(Q.getY())) {
	        f = getY() / Q.getY();
	    } else if (!Utility.isZero(Q.getOmega())) {
	        f = getOmega() / Q.getOmega();
	    }
	    return Utility.absEqual(getX(), f * Q.getX()) &&
	           Utility.absEqual(getY(), f * Q.getY()) &&
	           Utility.absEqual(getOmega(), f * Q.getOmega());
	}
	
	/**
	 * String representation of the homogeneous point
	 */
	@Override
	public String toString() {
		return x + " " + y + " " + omega;
	}
	
	/**
	 * Return the distance between two homogeneous points
	 * @param p
	 * @return the distance between two homogeneous points
	 */
	public double distance(Homogeneous p) {
		return toPoint().distance(p.toPoint());
	}
}
