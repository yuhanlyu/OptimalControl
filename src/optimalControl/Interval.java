package optimalControl;

/**
 * A class representing an interval
 * @author yu-hanlyu
 *
 */
public class Interval {
	public static final Interval EMPTY_INTERVAL = new Interval(0, -1);
	private double begin;
	private double end;
	
	/**
	 * Construct an interval with begin and end
	 * @param begin
	 * @param end
	 */
	public Interval(double begin, double end) {
		this.begin = begin;
		this.end = end;
	}
	
	/**
	 * Return the begin point
	 * @return begin point
	 */
	public double getBegin() {
		return begin;
	}
	
	/**
	 * Return the end point
	 * @return end point
	 */
	public double getEnd() {
		return end;
	}
	
	/**
	 * Compute the intersection with i
	 * @param i Interval
	 * @return the intersection
	 */
	public Interval intersect(Interval i) {
		if (getEnd() < i.getBegin() || i.getEnd() < getBegin())
	        return new Interval(0, 0);
		return new Interval(Double.max(getBegin(), i.getBegin()), Double.min(getEnd(), i.getEnd()));
	}
	
	/**
	 * Test whether a value x is in the interval
	 * @param x
	 * @return true if x is in the interval, otherwise false
	 */
	public boolean contains(double x) {
		return x >= begin && x <= end;
	}
	
	/**
	 * Test whether an interval is empty
	 * @return true if the interval is empty
	 */
	public boolean isEmpty() {
		return begin > end;
	}
	
	/**
	 * Return the length of a non-empty interval 
	 * @return
	 */
	public double length() {
		return end - begin;
	}
	
	/**
	 * String representation
	 */
	@Override
	public String toString() {
		return begin + " " + end;
	}
}
