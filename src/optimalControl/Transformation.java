package optimalControl;
import java.awt.geom.Point2D;

/**
 * 
 * @author yu-hanlyu
 *
 */
public class Transformation {
	private double x;
	private double y;
	private double sin;
	private double cos;
	
	/**
	 * Construct from a configuration
	 * @param q a configuration
	 */
	public Transformation(Configuration q) {
		x = q.getX();
		y = q.getY();
		sin = Math.sin(q.getTheta());
		cos = Math.cos(q.getTheta());
	}
	
	/**
	 * Construct from a point
	 * @param p a point
	 */
	public Transformation(Point2D p) {
		x = p.getX();
		y = p.getY();
		cos = 1;
		sin = 0;
	}
	
	/**
	 * Construct from position and orientation
	 * @param arg_x x-coordinate
	 * @param arg_y y-coordinate
	 * @param arg_sin sin theta
	 * @param arg_cos cos theta
	 */
	public Transformation(double arg_x, double arg_y, double arg_sin, double arg_cos) {
		x = arg_x;
		y = arg_y;
		sin = arg_sin;
		cos = arg_cos;
	}
	
	/**
	 * Create a transformation from a control and its duration
	 * @param u control
	 * @param time duration
	 */
	public Transformation(Control u, double time) {
        double theta = u.getOmega() * time, sinc = sinc(theta), verc = verc(theta);
        x = time * (u.getVx() * sinc - u.getVy() * verc);
        y = time * (u.getVx() * verc + u.getVy() * sinc);
        sin = Math.sin(theta);
        cos = Math.cos(theta);
    }
	
	/**
	 * Create a transformation from a control line
	 * @param controlLine
	 */
	public Transformation(ControlLine controlLine) {
		this.x = 0.0;
		this.y = controlLine.getKtheta();
		this.sin = -controlLine.getKy();
		this.cos = controlLine.getKx();
	}
	
	/**
	 * @param T the transformation to be transformed
	 * @return a combined transformation
	 */
	public Transformation transform(Transformation T) {
		return new Transformation(cos * T.getX() - sin * T.getY() + x,
				                  sin * T.getX() + cos * T.getY() + y,
				                  sin * T.getCos() + cos * T.getSin(),
				                  cos * T.getCos() - sin * T.getSin());
	}
	
	/**
	 * @param p the homogeneous point to be transformed
	 * @return the result homogeneous point
	 */
	public Homogeneous transform(Homogeneous p) {
		return new Homogeneous(cos * p.getX() - sin * p.getY() + x * p.getOmega(),
				               sin * p.getX() + cos * p.getY() + y * p.getOmega(),
				               p.getOmega());
	}
	
	/**
	 * Integrate a trajectory
	 * @param traj Trajectory want to integrate
	 * @return final configuration
	 */
	public Transformation move(Trajectory trajectory) {
		Transformation T = this;
		for (int i = 0; i < trajectory.size(); ++i) {
			T = T.move(trajectory.getControl(i), trajectory.getDuration(i));
		}
	    return T;
	}
	
	/**
	 * Integrate a trajectory with time limit
	 * @param traj Trajectory want to integrate
	 * @param time End time
	 * @return final configuration
	 */
	public Transformation move(Trajectory trajectory, double time) {
		double sum = 0.0;
		Transformation T = this;
		for (int i = 0; i < trajectory.size(); ++i) {
			if (time >= sum && time < sum + trajectory.getDuration(i)) {
				T = T.move(trajectory.getControl(i), time - sum);
				break;
			}
			T = T.move(trajectory.getControl(i), trajectory.getDuration(i));
			sum += trajectory.getDuration(i);
		}
	    return T;
	}
	
	/**
	 * Integrate a pair of control and time
	 * @param T transformation of the initial configuration
	 * @param controlTime a control with time
	 * @return a new Transformation of the final configuration
	 */
	public Transformation move(ControlTime controlTime) {
		return move(controlTime.getControl(), controlTime.getTime());
	}
	
	/**
	 * Integrate a control with a duration
	 * @param u: control want to apply
	 * @param time: duration of the control
	 * @return a new Transformation of the final configuration
	 */
	public Transformation move(Control u, double time) {
		return this.transform(new Transformation(u, time));
	}
	
	/**
	 * Return the transformation reflection to the control line
	 * @return the transformation reflection to the control line
	 */
	public Transformation reflect() {
		return new Transformation(-getX(), -getY(), -getSin(), -getCos());
	}
	
	
	/**
	 * Get the location
	 * @return a point in the plane
	 */
	public Point2D toPoint() {
		return new Point2D.Double(getX(), getY());
	}
	
	/**
	 * @return x-coordinate
	 */
	public double getX() {
		return x;
	}
	
	/**
	 * @return y-coordinate
	 */
	public double getY() {
		return y;
	}
	
	/**
	 * @return sin value
	 */
	public double getSin() {
		return sin;
	}
	
	/**
	 * @return cos value
	 */
	public double getCos() {
		return cos;
	}
	
	/**
	 * @return theta
	 */
	public double getTheta() {
		return Math.atan2(getSin(), getCos());
	}
	
	/**
	 * @param theta
	 * @return sinc value
	 */
	private static double sinc(double theta) {
		if (Math.abs(theta) < Utility.EPSILON) 
			return 1.0;
		return Math.sin(theta) / theta;
	}
	
	/**
	 * @param theta
	 * @return verc value
	 */
	private static double verc(double theta) {
		if (Math.abs(theta) < Utility.EPSILON) 
			return 0.0;
		return (1.0- Math.cos(theta))/theta;
	}
	
	public boolean close(Transformation T) {
		return Utility.absEqual(getX(), T.getX())
			&& Utility.absEqual(getY(),  T.getY())
			&& Utility.angleEqual(getTheta(), T.getTheta());
	}
	
	@Override
	public String toString() {
		return x + " " + y + " " + sin + " " + cos;
	}
	
	@Override
	public boolean equals(Object o) {
		Transformation T = (Transformation)o;
		return x == T.getX() && y == T.getY()
		    && cos == T.getCos() && sin == T.getSin();
	}
	
	@Override
	public int hashCode() {
		return new Double(x).hashCode()
			 ^ new Double(y).hashCode()
		     ^ new Double(cos).hashCode()
		     ^ new Double(sin).hashCode();
	}
}
