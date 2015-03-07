package rigidBody2DFreeSwitch;
import java.util.List;
import java.util.stream.Stream;

import optimalControl.ControlLine;
import optimalControl.Interval;
import optimalControl.Trajectory;


public class UniformSampleMinimizer extends DistanceMinimizer {
	private static final double DEFAULT_DELTA = 0.000001;
	private static final double DEFAULT_TIME_ERROR = 0.01;
	private static final double DEFAULT_DIST_ERROR = 2;
	private static final double STEP = 0.001;
	
	/**
	 * 
	 * @param minimization
	 */
	public UniformSampleMinimizer() {
		super(DEFAULT_DELTA, DEFAULT_DIST_ERROR, DEFAULT_TIME_ERROR);
	}
	
	/**
	 * 
	 * @param minimization
	 * @param delta
	 * @param distError
	 * @param timeError
	 */
	public UniformSampleMinimizer(double delta, double distError, double timeError) {
		super(delta, distError, timeError);
	}
	
	/**
	 * 
	 */
	@Override
	public GenericInfo minimize(DistanceMinimization minimization, boolean findAllTrajectories, List<TrajectoryInfo> trajectories) {
		Interval range = minimization.getRange();
		Stream.Builder<Double> builder = Stream.builder();
		for (double H = range.getBegin() + delta; H < range.getEnd() - delta; H += STEP) {
			builder.accept(H);
		}
		GenericInfo minSolution = builder.build()
				                                           .reduce(FreeGenericInfo.INFINITY, 
				                                                   (currentMin, H) -> 
				                                                   {  GenericInfo sol = minimization.computeTrajectory(H); 
				                                                      return compareSolution(sol, currentMin) < 0 ? sol : currentMin; },
				                                                   (solution1, solution2) -> compareSolution(solution1, solution2) < 0 ? solution1 : solution2);
		return minSolution.getDistanceTime().getDistance() < distError ? minSolution : FreeGenericInfo.INFINITY;
	}
	
	/**
	 * This function is used for animation
	 */
	@Override
	public GenericInfo minimize(DistanceMinimization minimization, List<Trajectory> trajList, List<ControlLine> controlLineList) {
		Interval range = minimization.getRange();
		System.out.println(range);
		for (double H = range.getBegin() + delta; H < range.getEnd() - delta; H += STEP) {
			GenericInfo sol = minimization.computeTrajectory(H);
			Trajectory trajectory = sol.getTrajectory();
			ControlLine controlLine = sol.getControlLine();
			trajList.add(trajectory);
			controlLineList.add(controlLine);
		}
		return minimize(minimization, false, null);
	}
}
