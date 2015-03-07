package rigidBody2DCostlySwitch;

import java.util.ArrayList;
import java.util.List;

import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.Utility;
import rigidBody2DFreeSwitch.FeasibleSolver;
import rigidBody2DFreeSwitch.GenericSolver;
import rigidBody2DFreeSwitch.LipschitzianMinimizer;
import rigidBody2DFreeSwitch.SingularSolver;
import rigidBody2DFreeSwitch.TGTSolver;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import rigidBody2DFreeSwitch.WhirlSolver;

public class CostlyPlanner {
	private static final double ERROR = 0.001;
	private static final double FEASIBLE_FACTOR = 5.0;
	private double switchCost;
	
	public CostlyPlanner(double switchCost) {
		this.switchCost = switchCost;
	}
	
	public TrajectoryInfo solve(ControlSet U, Transformation Ts) {
		TrajectoryInfo.setSwitchCost(switchCost);
		FeasibleSolver feasibleSolver = new FeasibleSolver(U, Ts);
		TrajectoryInfo feasibleSolution = feasibleSolver.solve();
		Trajectory feasibleTrajectory = feasibleSolution.getTrajectory();
		double upperBound = feasibleTrajectory.getCost(switchCost);
		TGTSolver TGTSolver = new CostlyTGTSolver(U, Ts, switchCost, upperBound);
		TrajectoryInfo TGTSolution = TGTSolver.solve();
		TrajectoryInfo minSolution = TGTSolution;
		WhirlSolver whirlSolver = new CostlyWhirlSolver(U, Ts, switchCost, upperBound);
		TrajectoryInfo whirlSolution = whirlSolver.solve();
		if (whirlSolution.compareSolution(minSolution) < 0 || whirlSolution.close(minSolution))
			minSolution = whirlSolution;
		SingularSolver singularSolver = new CostlySingularSolver(U, Ts, switchCost, upperBound);
		TrajectoryInfo singularSolution = singularSolver.solve();
		if (singularSolution.compareSolution(minSolution) < 0 || singularSolution.close(minSolution))
			minSolution = singularSolution;
		LipschitzianMinimizer minimizer = new LipschitzianMinimizer(ERROR, ERROR, 0.001);
		minimizer.setIncomplete(true);
		GenericSolver genericSolver = new CostlyGenericSolver(U, Ts, minimizer, switchCost, upperBound);
		TrajectoryInfo genericSolution = genericSolver.solve();
		if (genericSolution.compareSolution(minSolution) < 0)
			minSolution = genericSolution;
		return minSolution;
	}
	
	public List<TrajectoryInfo> getAllTrajectories(ControlSet U, Transformation Ts) {
		TrajectoryInfo.setSwitchCost(switchCost);
		FeasibleSolver feasibleSolver = new FeasibleSolver(U, Ts);
		TrajectoryInfo feasibleSolution = feasibleSolver.solve();
		System.out.println("Found feasible");
		Trajectory feasibleTrajectory = feasibleSolution.getTrajectory();
		double upperBound = feasibleTrajectory.getCost(switchCost) + FEASIBLE_FACTOR;
		List<TrajectoryInfo> solutions = new ArrayList<>();
		TGTSolver TGTSolver = new CostlyTGTAllSolver(U, Ts, switchCost, upperBound);
		solutions.addAll(TGTSolver.getAllTrajectories());
		System.out.println("Found TGT");
		WhirlSolver whirlSolver = new CostlyWhirlAllSolver(U, Ts, switchCost, upperBound);
		solutions.addAll(whirlSolver.getAllTrajectories());
		System.out.println("Found whirl");
		SingularSolver singularSolver = new CostlySingularAllSolver(U, Ts, switchCost, upperBound);
		solutions.addAll(singularSolver.getAllTrajectories());
		System.out.println("Found singular");
		LipschitzianMinimizer minimizer = new LipschitzianMinimizer(Utility.EPSILON, ERROR, ERROR);
		minimizer.setIncomplete(true);
		GenericSolver genericSolver = new CostlyGenericAllSolver(U, Ts, minimizer, switchCost, upperBound);
		solutions.addAll(genericSolver.getAllTrajectories());
		return solutions;
	}
}
