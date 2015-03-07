package rigidBody2DCostlySwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.Configuration;
import optimalControl.ControlLine;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import rigidBody2DFreeSwitch.DistanceMinimization;
import rigidBody2DFreeSwitch.DistanceMinimizer;
import rigidBody2DFreeSwitch.GenericInfo;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import robotModel.OmniDrive;

public class CostlyGenericAllSolver extends CostlyGenericSolver {
	private List<TrajectoryInfo> trajectories;
	
	public CostlyGenericAllSolver(ControlSet U, Transformation Ts,
			DistanceMinimizer minimizer, double switchCost,
			double upperBound) {
		super(U, Ts, minimizer, switchCost, upperBound);
	}
	
	@Override
	public List<TrajectoryInfo> getAllTrajectories() {
		trajectories = new ArrayList<>();
		solve();
		return trajectories;
	}
	
	@Override
	protected void processTask(DistanceMinimization task) {
		System.out.println("process");
		if (animate == true) {
			List<Trajectory> trajList = new ArrayList<>();
			List<ControlLine> controlLineList = new ArrayList<>();
			minimizer.minimize(task, trajList, controlLineList);
			minimizer.saveFile(new Configuration(Ts), new OmniDrive(), trajList, controlLineList);
		}
		GenericInfo solutionInfo = minimizer.minimize(task, true, trajectories);
		System.out.println(Ts + " " + Tf);
		if (minimizer.compareSolution(solutionInfo, optimalSolutionInfo) < 0) {
			optimalSolutionInfo = solutionInfo;
		}
	}
}
