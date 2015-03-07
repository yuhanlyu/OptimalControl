package rigidBody2DFreeSwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.ControlSet;
import optimalControl.Transformation;

public class TGTAllSolver extends TGTSolver {
	private List<TrajectoryInfo> trajectories;
	
	public TGTAllSolver(ControlSet U, Transformation Ts) {
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
