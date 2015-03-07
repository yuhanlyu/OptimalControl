package optimalControl;
/**
 * A class storing a transformation with time
 * @author yu-hanlyu
 *
 */
public class TransformationTime {
	private Transformation T;
	private double time;
	
	/**
	 * A pair of transformation and time
	 * @param T
	 * @param time
	 */
	public TransformationTime(Transformation T, double time) {
		this.T = T;
		this.time = time;
	}
	
	/**
	 * return the transformation
	 * @return
	 */
	public Transformation getTransformation() {
		return T;
	}
	
	/**
	 * return the time
	 * @return
	 */
	public double getTime() {
		return time;
	}
}
