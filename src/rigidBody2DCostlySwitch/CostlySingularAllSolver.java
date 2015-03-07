package rigidBody2DCostlySwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.ControlSet;
import optimalControl.Transformation;
import rigidBody2DFreeSwitch.TrajectoryInfo;

public class CostlySingularAllSolver extends CostlySingularSolver {
	private List<TrajectoryInfo> trajectories;
	
	public CostlySingularAllSolver(ControlSet U, Transformation Ts,
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