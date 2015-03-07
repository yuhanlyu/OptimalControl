package rigidBody2DFreeSwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.ControlSet;
import optimalControl.Transformation;

public class WhirlAllSolver extends WhirlSolver {
	private List<TrajectoryInfo> trajectories;
	public WhirlAllSolver(ControlSet U, Transformation Ts) {
		super(U, Ts);
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
