package rigidBody2DFreeSwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.ControlSet;
import optimalControl.Transformation;

public class SingularAllSolver extends SingularSolver {
	private List<TrajectoryInfo> trajectories;
	
	/**
	 * Constructor
	 * @param U a control set
	 * @param Ts the initial configuration
	 * @param upperBound upper bound of the cost of trajectories being searched
	 */
	public SingularAllSolver(ControlSet U, Transformation Ts, double upperBound) {
		super(U, Ts, upperBound);
	}

	@Override
	public List<TrajectoryInfo> getAllTrajectories() {
		trajectories = new ArrayList<>();
		solve();
		return trajectories;
	}

	@Override
	protected void foundOneSolution(TrajectoryInfo info) {
		trajectories.add(info);
	}
}
