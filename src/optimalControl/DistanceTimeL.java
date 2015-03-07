package optimalControl;
/**
 * A class storing Lipschitz constant for distance and time
 * @author yu-hanlyu
 *
 */
public class DistanceTimeL {
	private double distanceL;
	private double timeL;
	
	/**
	 * Constructor
	 * @param distanceL
	 * @param timeL
	 */
	public DistanceTimeL(double distanceL, double timeL) {
		this.distanceL = distanceL;
		this.timeL = timeL;
	}
	
	/**
	 * Return distance Lipschitz constant
	 * @return
	 */
	public double getDistanceL() {
		return distanceL;
	}
	
	/**
	 * Return time Lipschitz constant
	 * @return
	 */
	public double getTimeL() {
		return timeL;
	}
	
	@Override
	public String toString() {
		return String.format("%f %f", distanceL, timeL);
	}
}
