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
	 * @param T a transformtaion
	 * @param time duration
	 */
	public TransformationTime(Transformation T, double time) {
		this.T = T;
		this.time = time;
	}
	
	/**
	 * return the transformation
	 * @return the transformation
	 */
	public Transformation getTransformation() {
		return T;
	}
	
	/**
	 * return the time
	 * @return time
	 */
	public double getTime() {
		return time;
	}
}
