package optimalControl;


/**
 * @author Yu-Han Lyu
 *
 */
public class Configuration {
	private double x; // x-coordinate
	private double y; // y-coordinate
	private double theta; // orientation
	
	/**
	 * @param arg_x: x-coordinate
	 * @param arg_y: y-coordinate
	 * @param arg_theta: orientation
	 */
	public Configuration(double arg_x, double arg_y, double arg_theta){
		x = arg_x;
		y = arg_y;
		theta = arg_theta;
	}
	
	/**
	 * 
	 * @param q: another configuration
	 */
	public Configuration(Configuration q) {
		x = q.getX();
		y = q.getY();
		theta = q.getTheta();
	}
	
	/**
	 * Create a configuration corresponding to a transformation
	 * @param T
	 */
	public Configuration(Transformation T) {
		x = T.getX();
		y = T.getY();
		theta = T.getTheta();
	}
	
	/**
	 * @return return x value
	 */
	public double getX() {
		return x;
	}
	
	/**
	 * @return return y value
	 */
	public double getY() {
		return y;
	}
	
	/**
	 * @return return orientation
	 */
	public double getTheta() {
		return theta;
	}
	
	/**
	 * String representation of the configuration
	 */
	@Override
	public String toString() {
		return x + " " + y + " " + theta;
	}
}
