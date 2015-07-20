package rigidBody2DFreeSwitch;
import java.util.List;

import optimalControl.DistanceTime;
import optimalControl.DistanceTimeL;
import optimalControl.Interval;

/**
 * Task for a generic trajectory
 * @author yu-hanlyu
 *
 */
public class DistanceMinimization implements Comparable<DistanceMinimization> {
	private DistanceFunctor metric;
	private Interval range;
	private DistanceTime distanceTime;  // The estimated distance and time
		
	/**
	 * Constructor
	 * @param metric distance functor
	 * @param range the range of searching
	 * @param distanceTime estimated distance and time
	 */
	public DistanceMinimization(DistanceFunctor metric, 
			Interval range, DistanceTime distanceTime) {
		this.metric = metric;
		this.range = range;
		this.distanceTime = distanceTime;
	}

	/**
	 * Return the estimated distance and time
	 * @return the estimated distance and time
	 */
	public DistanceTime getEstimatedDistanceTime() {
		return distanceTime;
	}

	/**
	 * Return solution with respect to the Hamiltonian value H
	 * @param H the Hamiltonian value
	 * @return the distance and time
	 */
	public GenericInfo computeTrajectory(double H) {
		return metric.computeTrajectory(H);
	}
	
	/**
	 * Compute all solutions with respect to the Hamiltonian vale
	 * @param H the Hamiltonian value
	 * @param distError upper bound of the distance error
	 * @return all trajectories
	 */
	public List<GenericInfo> computeAllTrajectories(double H, double distError) {
		return metric.computeAllTrajectories(H, distError);
	}
	
	/**
	 * Return the distance and time with respect to the Hamiltonian value H
	 * @param H the Hamiltonian value
	 * @return the distance and time
	 */
	public DistanceTime computeDistanceTime(double H) {
		return metric.computeTrajectory(H).getDistanceTime();
	}
	
	/**
	 * Return the number of cycles for the previous solution
	 * @return number of cycles
	 */
	public long getCycles() {
		return metric.getCycles();
	}
	
	/**
	 * Return the Lipschitz constant for the Hamiltonian value H
	 * @param H the Hamiltonian value
	 * @return Lipschitz constant for distance and time
	 */
	public DistanceTimeL getL(double H, long multiplier) {
		return metric.getL(H, multiplier);
	}

	/**
	 * Return the range of the Hamiltonian values
	 * @return the range of the Hamiltonian values
	 */
	public Interval getRange() {
		return range;
	}

	/**
	 * Get the distance functor
	 * @return distance functor
	 */
	public DistanceFunctor getMetric() {
		return metric;
	}

	/**
	 * 
	 */
	@Override
	public int compareTo(DistanceMinimization o) {
		return getEstimatedDistanceTime().compareTo(o.getEstimatedDistanceTime());
	}
	
	/**
	 * Test whether there are multiple solutions
	 * @return true if there are multiple solutions
	 */
	public boolean hasMultipleSolution() {
		return metric.hasMultipleSolution();
	}
}