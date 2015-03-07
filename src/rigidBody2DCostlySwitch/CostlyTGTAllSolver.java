package rigidBody2DCostlySwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.ControlSet;
import optimalControl.Transformation;
import rigidBody2DFreeSwitch.TrajectoryInfo;

public class CostlyTGTAllSolver extends CostlyTGTSolver {
	private List<TrajectoryInfo> trajectories;
	public CostlyTGTAllSolver(ControlSet U, Transformation Ts,
			double switchCost, double upperBound) {
		super(U, Ts, switchCost, upperBound);
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
