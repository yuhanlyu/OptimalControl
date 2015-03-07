package rigidBody2DFreeSwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.Configuration;
import optimalControl.ControlLine;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import robotModel.OmniDrive;

/**
 * Find all generic trajectories that satisfy necessary conditions for optimal trajectories
 * @author yu-hanlyu
 *
 */
public class GenericAllSolver extends GenericSolver {
	private List<TrajectoryInfo> trajectories;
	
	public GenericAllSolver(ControlSet U, Transformation Ts,
			DistanceMinimizer minimizer) {
		super(U, Ts, minimizer);
	}

	/**
	 * Process one task with animation
	 * Here, I just hard code the robot
	 * Moreover, if there are multiple tasks, then only the result of the last one will be stored
	 * @param task
	 */
	@Override
	protected void processTask(DistanceMinimization task) {
		if (animate == true) {
			List<Trajectory> trajList = new ArrayList<>();
			List<ControlLine> controlLineList = new ArrayList<>();
			minimizer.minimize(task, trajList, controlLineList);
			minimizer.saveFile(new Configuration(Ts), new OmniDrive(), trajList, controlLineList);
		}
		GenericInfo solutionInfo = minimizer.minimize(task, true, trajectories);
		if (minimizer.compareSolution(solutionInfo, optimalSolutionInfo) < 0) {
			optimalSolutionInfo = solutionInfo;
		}
	}
	
	@Override
	public List<TrajectoryInfo> getAllTrajectories() {
		trajectories = new ArrayList<>();
		solve();
		return trajectories;
	}
}
