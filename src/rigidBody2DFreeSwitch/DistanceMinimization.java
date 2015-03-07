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
	 * @param metric
	 * @param lb
	 * @param ub
	 * @param distanceTime
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
	 * @return
	 */
	public long getCycles() {
		return metric.getCycles();
	}
	
	/**
	 * Return the Lipschitz constant for the Hamiltonian value H
	 * @param H
	 * @return
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
	 * 
	 * @return
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
	
	public boolean hasMultipleSolution() {
		return metric.hasMultipleSolution();
	}
}