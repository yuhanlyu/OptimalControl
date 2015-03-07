package optimalControl;
/**
 * A pair of distance and time
 * @author yu-hanlyu
 *
 */
public class DistanceTime implements Comparable<DistanceTime> {
	public static final DistanceTime INFINITY = new DistanceTime(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	private double distance;
	private double time;
	
	/**
	 * Constructor
	 * @param distance
	 * @param time
	 */
	public DistanceTime(double distance, double time) {
		this.distance = distance;
		this.time = time;
	}
	
	/**
	 * Return the distance
	 * @return the distance
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Return the time
	 * @return the time
	 */
	public double getTime() {
		return time;
	}

	@Override
	public int compareTo(DistanceTime o) {
		if (getDistance() < o.getDistance())
			return -1;
		else if (getDistance() > o.getDistance())
			return 1;
		else if (getTime() < o.getTime())
			return -1;
		else if (getTime() > o.getTime())
			return 1;
		return 0;
	}
	
	@Override
	public String toString() {
		return String.format("%f %f", distance, time);
	}
}
