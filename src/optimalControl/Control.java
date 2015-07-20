package optimalControl;
import java.awt.geom.Point2D;

/**
 * @author yu-hanlyu
 *
 */
public class Control {
	private double vx; // velocity in x-coordinate
	private double vy; // velocity in y-coordinate
	private double omega; // angular velocity
	
	/**
	 * @param arg_vx velocity in x-coordinate
	 * @param arg_vy velocity in y-coordinate
	 * @param arg_omega angular velocity
	 */
	public Control(double arg_vx, double arg_vy, double arg_omega) {
		this.vx = arg_vx;
		this.vy = arg_vy;
		this.omega = arg_omega;
	}
	
	/**
	 * Create a control from a homogeneous point
	 * @param p a Homogeneous point
	 */
	public Control(Homogeneous p) {
		this.vx = p.getY();
		this.vy = -p.getX();
		this.omega = p.getOmega();
	}
	
	/**
	 * Get the reverse of the control
	 * @return reverse control
	 */
	public Control reverse() {
		return new Control(-getVx(), -getVy(), -getOmega());
	}
	
	/**
	 * @return the angular velocity
	 */
	public double getOmega() {
		return omega;
	}

	/**
	 * @return the velocity in y-coordinate
	 */
	public double getVy() {
		return vy;
	}
	
	/**
	 * @return the velocity in x-coordinate
	 */
	public double getVx() {
		return vx;
	}
	
	/**
	 * @return true if the control is a rotation
	 */
	public boolean isRotation() {
		return omega != 0;
	}
	
	/**
	 * test whether is identical to u
	 * @param u control
	 * @return true if this and u are identical
	 */
	@Override
	public boolean equals(Object o) {
		Control u = (Control) o;
		return vx == u.getVx() && vy == u.getVy() && omega == u.getOmega();
	}
	
	/**
	 * Test two controls are very close
	 * @param u a contorl
	 * @return true if this and u are close
	 */
	public boolean close(Control u) {
		return Utility.absEqual(vx, u.getVx())
			&& Utility.absEqual(vy, u.getVy())
			&& Utility.absEqual(omega, u.getOmega());
	}
	
	/**
	 * Return the velocity of the control
	 * @return velocity
	 */
	public double getVelocity() {
		return Math.sqrt(vx * vx + vy * vy);
	}
	
	/**
	 * Compute the switch point to u
	 * @param u control 
	 * @return switch point represented as a homogeneous point
	 */
	public Homogeneous switchPoint(Control u) {
		return new Homogeneous(getVy() - u.getVy(), u.getVx() - getVx(), u.getOmega() - getOmega());
	}
	
	/**
	 * String representation of control
	 */
	@Override
	public String toString() {
		return vx + " " + vy + " " + omega;
	}
	
	/**
	 * Test whether the control is a translation
	 * @return
	 */
	public boolean isTranslation() {
		return !isRotation();
	}
	
	/**
	 * Return the distance between the rotation centers of two controls
	 * @param u
	 * @return
	 */
	public double distance(Control u) {
		return (new Homogeneous(this)).distance(new Homogeneous(u));
	}
	
	/**
	 * Compute the control in the world frame after applied transformation T
	 * @param T
	 * @return
	 */
	public Control toWorld(Transformation T) {
		return new Control(T.transform(new Homogeneous(this)));
	}
	
	/**
	 * Compute the rotation center in the local frame
	 * @return the rotation center in the local frame
	 */
	public Point2D rotationCenter() {
		return new Point2D.Double(-getVy() / getOmega(), getVx() / getOmega());
	}
	
	/**
	 * Compute the rotation center of control u attaching a configuration in the world frame
	 * @param T a configuration
	 * @return the rotation in the world frame
	 */
	public Point2D rotationCenter(Transformation T) {
		return T.transform(new Homogeneous(this)).toPoint();
	}
	
	/**
	 * Compute the radius of rotation
	 * @return the radius of the rotation
	 */
	public double radius() {
		if (isTranslation())
			return Double.POSITIVE_INFINITY;
		return Utility.distanceToOrigin(rotationCenter());
	}
	
	@Override
	public int hashCode() {
		return new Double(vx).hashCode() ^ new Double(vy).hashCode() ^ new Double(omega).hashCode();
	}
}