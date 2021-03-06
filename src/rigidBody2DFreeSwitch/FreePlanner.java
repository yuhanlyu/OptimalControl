package rigidBody2DFreeSwitch;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;

/**
 * 
 * @author yu-hanlyu
 *
 */
public class FreePlanner {
	private HashMap<String, TrajectoryInfo> solutionMap;
	private static final Logger logger = Logger.getLogger(FreePlanner.class.getName());
	/**
	 * Constructor 
	 */
	public FreePlanner() {
	}
	
	/**
	 * Determine the optimal trajectory by trying all possible trajectory types
	 * @param U a control set
	 * @param Ts initial configuration
	 * @param Tf goal configuraiton
	 * @return  a trajectory
	 */
	public TrajectoryInfo solve(ControlSet U, Transformation Ts) {
		logger.info("Start to solve, initial configuration is " + Ts);
		solutionMap = new HashMap<>();
		FeasibleSolver feasibleSolver = new FeasibleSolver(U, Ts);
		TrajectoryInfo feasibleSolution = feasibleSolver.solve();
		solutionMap.put("Feasible", feasibleSolution);
		logger.info("Searching feasible solution completed");
		TrajectoryInfo minSolution = feasibleSolution;
		TGTSolver TGTSolver = new TGTSolver(U, Ts);
		TrajectoryInfo TGTSolution = TGTSolver.solve();
		solutionMap.put("TGT", TGTSolution);
		if (TGTSolution.compareSolution(minSolution) < 0 || TGTSolution.close(minSolution))
			minSolution = TGTSolution;
		logger.info("Searching TGT solutions completed");
		
		WhirlSolver whirlSolver = new WhirlSolver(U, Ts);
		TrajectoryInfo whirlSolution = whirlSolver.solve();
		solutionMap.put("Whirl", whirlSolution);
		if (whirlSolution.compareSolution(minSolution) < 0 || whirlSolution.close(minSolution))
			minSolution = whirlSolution;
		logger.info("Searching whirl solutions completed");
		SingularSolver singularSolver = new SingularSolver(U, Ts, minSolution.getTime());
		TrajectoryInfo singularSolution = singularSolver.solve();
		solutionMap.put("Singular", singularSolution);
		if (singularSolution.compareSolution(minSolution) < 0 || singularSolution.close(minSolution))
			minSolution = singularSolution;
		logger.info("Searching singular solutions completed");
		
		//genericSolver = new GenericTrajectorySolver(U, Ts,  new UniformSampleMinimizer());
		GenericSolver genericSolver = new GenericSolver(U, Ts,  new LipschitzianMinimizer(1e-3, 1e-3, 1e-3));
		TrajectoryInfo genericSolution = genericSolver.solve();
		logger.info("Searching generic solutions completed");
		solutionMap.put("Generic", genericSolution);
		if (genericSolution.compareSolution(minSolution) < 0)
			minSolution = genericSolution;
		solutionMap.put("Optimal", minSolution);
		return minSolution;
	}
	
	/**
	 * Return solutions 
	 * @param type type of the trajectory
	 * @return a trajectory
	 */
	public TrajectoryInfo getSolution(String type) {
		return solutionMap.get(type);
	}
	
	/**
	 * Return all trajectories found by the planner
	 * @param U the control set
	 * @param Ts the initial configuration
	 * @return all trajectories
	 */
	public List<TrajectoryInfo> getAllTrajectories(ControlSet U, Transformation Ts) {
		FeasibleSolver feasibleSolver = new FeasibleSolver(U, Ts);
		TrajectoryInfo feasibleSolution = feasibleSolver.solve();
		Trajectory feasibleTrajectory = feasibleSolution.getTrajectory();
		double upperBound = feasibleTrajectory.totalTime();
		List<TrajectoryInfo> solutions = new ArrayList<>();
		TGTSolver TGTSolver = new TGTAllSolver(U, Ts);
		solutions.addAll(TGTSolver.getAllTrajectories());
		WhirlSolver whirlSolver = new WhirlAllSolver(U, Ts);
		solutions.addAll(whirlSolver.getAllTrajectories());
		SingularSolver singularSolver = new SingularAllSolver(U, Ts, upperBound);
		solutions.addAll(singularSolver.getAllTrajectories());
		GenericSolver genericSolver = new GenericAllSolver(U, Ts, new LipschitzianMinimizer());
		solutions.addAll(genericSolver.getAllTrajectories());
		return solutions;
	}
}
