package rigidBody2DFreeSwitch;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import optimalControl.ControlLine;
import optimalControl.DistanceTime;
import optimalControl.DistanceTimeL;
import optimalControl.Interval;
import optimalControl.Trajectory;
import optimalControl.Utility;

/**
 * Lipschitzian minimizer
 * Although it works theoretically, there are several difficulties.
 * 1. The upper bound of the number of cycles: correct upper bound for Lipschitz constants needs
 * upper bound of the number of cycles. So far, there is no theoretical way to get the upper bound of cycles.
 * The heuristic I used here is to compute the cycle at the beginning and at the end of the range and then
 * pick the maximum.
 * 2. Numerical stability issues when H is close to critical values: When H is close to critical values,
 * the computation is unstable and may result in incorrect Lipschitz constants. Hence, picking a suitable
 * DELTA is necessary. In the program, I didn't check the correctness of Lipschitz constants.  
 * @author yu-hanlyu
 *
 */
public class LipschitzianMinimizer extends DistanceMinimizer {
	private static final double DEFAULT_DELTA = 50 * Utility.EPSILON;  // Need some distance away from critical values
	private static final double DEFAULT_TIME_ERROR = Utility.EPSILON;  // Some configurations may take very long time if
	                                                         		   // this value is too small
	private static final double DEFAULT_DIST_ERROR = Utility.EPSILON;
	private boolean incomplete = false;                               // When incomplete is true, program will skip small region
	  																  // This will speed up for some scenario but may not find the optimal solution
	private static final double MIN_REGION = 0.001;                   // In incomplete mode, when the region is smaller than MIN_REGION,
	 																  // the program will skip that region.
	private double feasibleTime = Double.POSITIVE_INFINITY;
	/**
	 * 
	 * @param minimization
	 */
	public LipschitzianMinimizer() {
		super(DEFAULT_DELTA, DEFAULT_DIST_ERROR, DEFAULT_TIME_ERROR);
	}
	
	/**
	 * Constructor
	 * @param delta gap along boundaries
	 * @param distError maximum distance error
	 * @param timeError maximum time error
	 */
	public LipschitzianMinimizer(double delta, double distError, double timeError) {
		super(delta, distError, timeError);
	}
	
	/**
	 * Set the incomplete mode
	 * @param mode
	 */
	public void setIncomplete(boolean mode) {
		incomplete = mode;
	}
	
	/**
	 * Search node class for Lipschitzian optimization
	 * @author yu-hanlyu
	 *
	 */
	private class Region implements Comparable<Region> {
		private Interval range;
		private DistanceTime beginDT;  // The distance and time for the begin of the range
		private DistanceTime endDT;    // The distance and time for the end of the range
		private DistanceTimeL L;       // Lipschitz constant for the range
		private DistanceTime lb;       // The lower bound for the distance and time for the range
		private long beginCycles;
		private long endCycles;
		
		/**
		 * Constructor
		 * @param range range of searching
		 * @param minimization minimization task
		 */
		public Region(Interval range, DistanceMinimization minimization) {
			this.range = range;
			this.beginDT = minimization.computeDistanceTime(range.getBegin());
			this.beginCycles = minimization.getCycles();
			this.endDT = minimization.computeDistanceTime(range.getEnd());
			this.endCycles = minimization.getCycles();
			this.L = minimization.getL(range.getEnd(), 1 + Long.max(beginCycles, endCycles));
			lb = lowerBound(beginDT, endDT, L, range);
		}
		
		/**
		 * Constructor
		 * @param range range of search
		 * @param beginDT distance and time at the beginning of the range
		 * @param endDT distance and time at the end of the range
		 * @param L Lipschitz constant in the range
		 */
		public Region(Interval range, DistanceTime beginDT, DistanceTime endDT, DistanceTimeL L, long beginCycles, long endCycles) {
			this.range = range;
			this.beginDT = beginDT;
			this.endDT = endDT;
			this.L = L;
			this.beginCycles = beginCycles;
			this.endCycles = endCycles;
			lb = lowerBound(beginDT, endDT, L, range);
		}
		
		/**
		 * Compute the split point for the current region
		 * @return the split point
		 */
		public double splitPoint() {
			double split = (range.getBegin() + range.getEnd()) * 0.5 + 
				           ((beginDT.getDistance() - endDT.getDistance()) * 0.5) / L.getDistanceL();
			// Sometimes the Lipschitz constant is wrong, especially when it is close to critical values
			if (!range.contains(split)) {
				split = (range.getBegin() + range.getEnd()) * 0.5;
			}
			// If didn't make any progress due to numerical error, just split in half
			if (split == range.getBegin() || split == range.getEnd()) {
				split = (range.getBegin() + range.getEnd()) * 0.5;
			}
			return split;
		}
		
		/**
		 * Split the region by the split point and return left part
		 * @param splitPoint the split point
		 * @param dt distance time for the whole range
		 * @param dtL Lipschitz constant for the whole range
		 * @param midCycles the number of cycles at the middle point
		 * @return left part
		 */
		public Region splitLeft(double splitPoint, DistanceTime dt, DistanceTimeL dtL, long midCycles) {
			return new Region(new Interval(range.getBegin(), splitPoint), beginDT, dt, dtL, beginCycles, midCycles);
		}
		
		/**
		 * Split the region by the split point and return right part
		 * @param splitPoint the split point
		 * @param dt distance time for the whole range
		 * @param dtL Lipschitz constant for the whole range
		 * @param midCycles the number of cycles at the middle point
		 * @return right part
		 */
		public Region splitRight(double splitPoint, DistanceTime dt, DistanceTimeL dtL, long midCycles) {
			return new Region(new Interval(splitPoint, range.getEnd()), dt, endDT, dtL, midCycles, endCycles);
		}
		
		/**
		 * Return the lower bound of the time for the region
		 * @return lower bound of the time for the region
		 */
		public double getTimeLowerBound() {
			return lb.getTime();
		}
		
		/**
		 * Return the lower bound of the distance for the region
		 * @return the lower bound of the distance for the region
		 */
		public double getDistanceLowerBound() {
			return lb.getDistance();
		}

		@Override
		public int compareTo(Region o) {
			return compareDistanceTime(lb, o.lb);
		}
		
		/**
		 * Compute the lower bound
		 * @param begin begin of the range
		 * @param end end of the range
		 * @param L Lipschitz constant
		 * @param interval interval
		 * @return lower bound on the distance and time
		 */
		private DistanceTime lowerBound(DistanceTime begin, DistanceTime end, DistanceTimeL L, Interval interval) {
			double distLb = (begin.getDistance() + end.getDistance() - interval.length() * L.getDistanceL()) * 0.5;
			double timeLb = (begin.getTime() + end.getTime() - interval.length() * L.getTimeL()) * 0.5;
			return new DistanceTime(distLb, timeLb);
		}
		
		/**
		 * Test whether the region is useful
		 * @param minDT minimum ditance and time so far
		 * @param findAllTrajectories whether wants to find all trajectories
		 * @param hasFound already explored before
		 * @param cycles number of cycles
		 * @return true if the region is useful, false otherwise
		 */
		public boolean isUsefulRegion(DistanceTime minDT, boolean findAllTrajectories, boolean hasFound, long cycles) {
			if (range.isEmpty())
				throw new RuntimeException("Range is empty");
			// Ignore some small area
			if (incomplete && range.length() < MIN_REGION)
				return false;
			if (getDistanceLowerBound() > distError)
				return false;
			if (!findAllTrajectories && getTimeLowerBound() > minDT.getTime() - timeError)
				return false;
			if (findAllTrajectories && hasFound && beginCycles == cycles && endCycles == cycles)
				return false;
			return true;
		}
	}
	
	/**
	 * Piyavskii algorithm
	 */
	@Override
	public GenericInfo minimize(DistanceMinimization minimization, boolean findAllTrajectories, List<TrajectoryInfo> trajectories) {
		Interval range = minimization.getRange();
		Interval safeRange = new Interval(range.getBegin() + delta, range.getEnd() - delta);
		if (safeRange.isEmpty())
			return FreeGenericInfo.INFINITY;
		Map<Long, TrajectoryInfo> results = new HashMap<>();
		DistanceTime minDT = new DistanceTime(0, feasibleTime);
		PriorityQueue<Region> queue = new PriorityQueue<>();
		queue.add(new Region(safeRange, minimization));
		double minH = Double.NaN;
		while (!queue.isEmpty()) {
			Region region = queue.remove();
			double splitPoint = region.splitPoint();
			DistanceTime dt = minimization.computeDistanceTime(splitPoint);
			//System.out.println("Split at: " + splitPoint + " " + dt.getDistance());
			long cycles = minimization.getCycles();
			boolean reachGoal = dt.getDistance() < distError;
			
			// For costly generic trajectories
			if (findAllTrajectories && minimization.hasMultipleSolution()) {
				List<GenericInfo> infos = minimization.computeAllTrajectories(splitPoint, distError);
				for (GenericInfo info : infos) {
					if (info == FreeGenericInfo.INFINITY)
						continue;
					results.put((long) results.size(), TrajectoryInfo.createGeneric(info.getTrajectory(), 
                               null,
                               info.getControlLine()));
				}
			// For free generic trajectories
			} else if (findAllTrajectories && results.containsKey(cycles) && dt.getDistance() < distError) {
				GenericInfo info = minimization.computeTrajectory(splitPoint);
				TrajectoryInfo sol = TrajectoryInfo.createGeneric(info.getTrajectory(), 
						                                          info.getStructure(),
						                                          info.getControlLine());
				results.put(cycles, sol);
			}
			
			if (compareDistanceTime(dt, minDT) < 0) {
				minDT = dt;
				minH = splitPoint;
			}
			// Here, we assume the number of cycles in monotone with respect to H
			// Although it is not true in general, it works in practice
			DistanceTimeL dtLLeft = minimization.getL(splitPoint, 1 + Long.max(region.beginCycles, cycles));
			DistanceTimeL dtLRight = minimization.getL(region.range.getEnd(), 1 + Long.max(cycles, region.endCycles));
			Region left = region.splitLeft(splitPoint, dt, dtLLeft, cycles);
			Region right = region.splitRight(splitPoint, dt, dtLRight, cycles);
			if (left.isUsefulRegion(minDT, findAllTrajectories, reachGoal, cycles)) {
				queue.add(left);
			}
			if (right.isUsefulRegion(minDT, findAllTrajectories, reachGoal, cycles)) {
				queue.add(right);
			}
		}
		if (findAllTrajectories && results.size() > 0) {
			trajectories.addAll(results.values());
		}
		return Double.isNaN(minH) ? FreeGenericInfo.INFINITY : minimization.computeTrajectory(minH);
	}

	/**
	 * This function is used for animation
	 */
	@Override
	public GenericInfo minimize(DistanceMinimization minimization, List<Trajectory> trajList, List<ControlLine> controlLineList) {
		Interval range = minimization.getRange();
		Interval safeRange = new Interval(range.getBegin() + delta, range.getEnd() - delta);
		if (safeRange.isEmpty())
			return FreeGenericInfo.INFINITY;
		DistanceTime minDT = new DistanceTime(0, feasibleTime);
		PriorityQueue<Region> queue = new PriorityQueue<>();
		queue.add(new Region(safeRange, minimization));
		double minH = Double.NaN;
		while (!queue.isEmpty()) {
			Region region = queue.remove();
			double splitPoint = region.splitPoint();
			DistanceTime dt = minimization.computeDistanceTime(splitPoint);
			long cycles = minimization.getCycles();
			GenericInfo sol = minimization.computeTrajectory(splitPoint);
			Trajectory trajectory = sol.getTrajectory();
			ControlLine controlLine = sol.getControlLine();
			trajList.add(trajectory);
			controlLineList.add(controlLine);
			if (compareDistanceTime(dt, minDT) < 0) {
				minDT = dt;
				minH = splitPoint;
			}
			DistanceTimeL dtLLeft = minimization.getL(splitPoint, 1 + Long.max(region.beginCycles, cycles));
			DistanceTimeL dtLRight = minimization.getL(region.range.getEnd(), 1 + Long.max(cycles, region.endCycles));
			Region left = region.splitLeft(splitPoint, dt, dtLLeft, cycles);
			Region right = region.splitRight(splitPoint, dt, dtLRight, cycles);
			if (left.isUsefulRegion(minDT, false, false, -1)) {
				queue.add(left);
			}
			if (right.isUsefulRegion(minDT, false, false, -1)) {
				queue.add(right);
			}
		}
		return minimize(minimization, false, null);
	}
}
