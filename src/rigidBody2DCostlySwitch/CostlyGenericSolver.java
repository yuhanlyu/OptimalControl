package rigidBody2DCostlySwitch;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import optimalControl.Configuration;
import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlLineFactory;
import optimalControl.ControlLineFactory.LFunctor;
import optimalControl.ControlSet;
import optimalControl.DTFunctor;
import optimalControl.DistanceTime;
import optimalControl.DistanceTimeL;
import optimalControl.Interval;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.TransformationTime;
import optimalControl.Utility;
import rigidBody2DFreeSwitch.DistanceFunctor;
import rigidBody2DFreeSwitch.DistanceMinimization;
import rigidBody2DFreeSwitch.DistanceMinimizer;
import rigidBody2DFreeSwitch.FreeGenericInfo;
import rigidBody2DFreeSwitch.GenericInfo;
import rigidBody2DFreeSwitch.GenericSolver;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import robotModel.OmniDrive;

/**
 * Finding optimal trajectories in the costly switch model
 * @author yu-hanlyu
 *
 */
public class CostlyGenericSolver extends GenericSolver {
	private double switchCost;
	private double vmax;
	private long maxSwitches;
	
	public CostlyGenericSolver(ControlSet U, Transformation Ts,
			                   DistanceMinimizer minimizer, double switchCost, double upperBound) {
		super(U, Ts, minimizer);
		this.switchCost = switchCost;
		vmax = U.maxVelocity();
		this.maxSwitches = (long)Math.ceil(upperBound / switchCost);
	}
	
	@Override
	public TrajectoryInfo solve() {
		animate = false;
		solution = TrajectoryInfo.INFINITY;
		optimalSolutionInfo = FreeGenericInfo.INFINITY;
		List<DistanceMinimization> tasks = generateTasks(); 
		generateTasks().stream()
		               .filter(task -> task != null)
		               .sorted()
		               .forEach(task -> processTask(task));
		if (optimalSolutionInfo != FreeGenericInfo.INFINITY) {
			solution = TrajectoryInfo.createGeneric(optimalSolutionInfo.getTrajectory(), 
			     	                                optimalSolutionInfo.getStructure(),
			     	                                optimalSolutionInfo.getControlLine());
			//if (!isGoal(Ts.move(solution.getTrajectory()))) {
			//	solution = TrajectoryInfo.INFINITY;
			//	optimalSolutionInfo = FreeGenericInfo.INFINITY;
			//}
			minTime = solution.getTrajectory().totalTime();
		}
		return solution;
	}
	
	@Override
	protected List<DistanceMinimization> generateTasks(Control us, Control uf) {
		ArrayList<DistanceMinimization> result = new ArrayList<>();
		ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, true);
		result.addAll(generateTasks(us, uf, factory));
		factory = new ControlLineFactory(U, Ts, us, Tf, uf, false);
		result.addAll(generateTasks(us, uf, factory));
		return result;
	}
	
	@Override
	protected List<DistanceMinimization> generateTasks(Control us, Control uf, ControlLineFactory factory) {
		double lb = 0.0, ub = Math.min(factory.getUpperBoundOfH(), U.getUpperBound());
		Interval range = new Interval(lb, ub);
		return generateTasks(us, uf, factory, range);
	}
	
	/**
	 * Generate a task with initial control us, final control uf, the sign of control line, and range of H 
	 * @param us first control
	 * @param uf last control
	 * @param isPositive the sign of the control line
	 * @param factory  factory of the control line
	 * @param lb lower bound of the range of H
	 * @param ub upper bound of the range of H
	 * @return a task with initial control us, final control uf, the sign of control line, and range of H
	 */
	@Override
	protected DistanceMinimization generateTask(Control us, Control uf, ControlLineFactory factory, double lb, double ub) {
		if (Utility.absEqual(lb, ub))
			return null;
		double H = (lb + ub) * 0.5;
		DistanceFunctor metric = new CostlySwitchDistanceFunctor(factory, H);
		if (!metric.hasStructure())
			return null;
		DistanceTime distanceTime = metric.computeTrajectory(H).getDistanceTime();
		System.out.println("generate");
		if (Double.isInfinite(distanceTime.getDistance())) {
			System.out.println("Error in generateTask");
		}
		return new DistanceMinimization(metric, new Interval(lb, ub), distanceTime);
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
		GenericInfo solutionInfo = minimizer.minimize(task);
		//System.out.println(Ts + " " + Tf);
		if (minimizer.compareSolution(solutionInfo, optimalSolutionInfo) < 0) {
			optimalSolutionInfo = solutionInfo;
		}
	}
	
	/**
	 * Distance functor class for costly switch model
	 * @author yu-hanlyu
	 *
	 */
	private class CostlySwitchDistanceFunctor implements DistanceFunctor {
		private List<LFunctor> sfunctors = new ArrayList<>();
		private List<LFunctor> ffunctors = new ArrayList<>();
		private List<DTFunctor> dtFunctors = new ArrayList<>();
		private List<GenericInfo> resultTrajectories = new ArrayList<>();
		private ControlLineFactory factory;
		private double minDist = Double.POSITIVE_INFINITY;
		private double minTime = Double.POSITIVE_INFINITY;
		private double distError;
		
		public CostlySwitchDistanceFunctor(ControlLineFactory factory, double H) {
			this.factory = factory;
			// For computing mapping Lipschitz continuous
			for (Control u : U) {
				if (!u.equals(getUs()) && (u.isRotation() || getUs().isRotation()))
					sfunctors.add(factory.getLFunctor(Ts, getUs(), u));
				if (!u.equals(getUf()) && (u.isRotation() || getUf().isRotation()))
					ffunctors.add(factory.getLFunctor(Tf, u, getUf()));
			}
			// For computing switching Lipschitz continuous
			for (Control pre : U) {
				for (Control u : U) {
					if (pre.equals(u) || (pre.isTranslation() && u.isTranslation()))
						continue;
					for (Control next : U) {
						if (next.equals(u) || (u.isTranslation() && next.isTranslation()))
							continue;
						DTFunctor functor = DTFunctor.create(pre, u, next);
						try {
							DistanceTimeL dtL = functor.getL(H);
							dtFunctors.add(functor);
						} catch (IllegalArgumentException e) {
							continue;
						}
					}
				}
			}
		}
		
		@Override
		public DistanceTimeL getL(double H, long multiplier) {
			DistanceTimeL beginL = computeMappingL(H, sfunctors);
			DistanceTimeL endL = computeMappingL(H, ffunctors);
			DistanceTimeL strucL = computeStructureL(H);
			return new DistanceTimeL(beginL.getDistanceL() + endL.getDistanceL() + strucL.getDistanceL() * maxSwitches,
					                 beginL.getTimeL() + endL.getTimeL() + strucL.getTimeL() * maxSwitches);
		}
		
		/**
		 * Finding the maximum of all possible switching Lipschitz constants
		 * @param H
		 * @return
		 */
		private DistanceTimeL computeStructureL(double H) {
			double distL = Double.NEGATIVE_INFINITY, timeL = Double.NEGATIVE_INFINITY;
			for (DTFunctor functor : dtFunctors) {					DistanceTimeL dtL = functor.getL(H);
				if (!Double.isFinite(dtL.getDistanceL()) || !Double.isFinite(dtL.getTimeL()))
					continue;
				if (dtL.getDistanceL() > distL)
					distL = dtL.getDistanceL();
				if (dtL.getTimeL() > timeL)
					timeL = dtL.getTimeL();
			}
			return new DistanceTimeL(distL, timeL);
		}
		
		/**
		 * Finding the maximum of all possible mapping Lipschitz constants
		 * @param H
		 * @param functors
		 * @return
		 */
		private DistanceTimeL computeMappingL(double H, List<LFunctor> functors) {
			double distL = Double.NEGATIVE_INFINITY, timeL = Double.NEGATIVE_INFINITY;
			for (LFunctor functor : functors) {
				DistanceTimeL dtL = functor.getL(H);
				if (!Double.isFinite(dtL.getDistanceL()) || !Double.isFinite(dtL.getTimeL()))
					continue;
				if (dtL.getDistanceL() > distL)
					distL = dtL.getDistanceL();
				if (dtL.getTimeL() > timeL)
					timeL = dtL.getTimeL();
				
			}
			return new DistanceTimeL(distL, timeL);
		}
		
		/**
		 * Return the first control
		 * @return the first control
		 */
		private Control getUs() {
			return factory.getUs();
		}
		
		/**
		 * Return the last control
		 * @return the last control
		 */
		private Control getUf() {
			return factory.getUf();
		}

		@Override
		public long getCycles() {
			return 1;
		}

		@Override
		public boolean hasStructure() {
			return true;
		}

		/**
		 * A* search to find the minimum cost trajectory with respect to the Hamiltonian value H
		 */
		@Override
		public GenericInfo computeTrajectory(double H) {
			return computeTrajectory(H, false).get(0);
		}
		
		/**
		 * Check whether the node can reach the destination
		 * @param node
		 * @param transformationTime
		 * @param dest
		 * @return
		 */
		private Trajectory checkGoal(SearchNode node, ControlLine controlLine, TransformationTime transformationTime, TransformationTime dest) {
			Transformation T = transformationTime.getTransformation();
			Transformation goal = dest.getTransformation();
			if (goal == null)
				return null;
			if (!Utility.absEqual(T.getY(), goal.getY()) || !Utility.angleEqual(T.getTheta(), goal.getTheta()))
				return null;
			double dist = Math.abs(T.getX() - goal.getX());
			double duration = transformationTime.getTime();
			double cost = node.cost + duration + 2.0 * switchCost + dest.getTime();
			Trajectory temp = new Trajectory();
			temp.addControl(node.next, duration);
			for ( ;node.cost > 0.0; node = node.parent) {
				temp.addControl(node.u, node.duration);
			}
			Trajectory result = new Trajectory();
			for (int i = temp.size() - 1; i >= 0; --i) {
				result.addControl(temp.getControl(i), temp.getDuration(i));
			}
			result.addControl(getUf(), dest.getTime());
			if (dist < distError)
				resultTrajectories.add(new CostlyGenericInfo(result, controlLine, new DistanceTime(cost, dist)));
			if (cost < minTime) {
				minTime = cost;
				minDist = dist;
				return result;
			}
			return null;
		}
		
		/**
		 * Check whether the node can reach the goal
		 * @param node
		 * @param transformationTime
		 * @param dest
		 * @return
		 */
		private Trajectory checkGoal(SearchNode node, Control u, ControlLine controlLine, TransformationTime transformationTime, HashMap<Control, TransformationTime[]> goalMap) {
			if (!getUf().equals(u) || !Double.isFinite(transformationTime.getTime()))
				return null;
			TransformationTime[] goalPair = goalMap.get(node.next);
			Trajectory traj1 = checkGoal(node, controlLine, transformationTime, goalPair[0]);
			Trajectory traj2 = checkGoal(node, controlLine, transformationTime, goalPair[1]);
			if (traj2 != null)
				return traj2;
			return traj1;
		}
		
		/**
		 * Initialize the goal map
		 * @param TfL
		 * @param goalMap
		 */
		private void initialMap(Transformation TfL, HashMap<Control, TransformationTime[]> goalMap) {
			for (Control u : U) {
				if (getUf().equals(u))
					continue;
				TransformationTime[] TransformationTimePair= U.completeSegment2Reverse(TfL, getUf(), u);
				goalMap.put(u, TransformationTimePair);
			}
		}

		@Override
		public List<GenericInfo> computeAllTrajectories(double H, double distError) {
			this.distError = distError;
			return computeTrajectory(H, true);
		}
		
		/**
		 * Find trajectories with respect to the Hamiltonian value H
		 * @param H
		 * @param findAllTrajectories
		 * @return
		 */
		private List<GenericInfo> computeTrajectory(double H, boolean findAllTrajectories) {
			ControlLine controlLine = factory.getControlLine(H);
			Transformation TLW = new Transformation(controlLine);
			Transformation TfL = TLW.transform(Tf);
			HashMap<Control, TransformationTime[]> goalMap = new HashMap<>();
			initialMap(TfL, goalMap);
			
			Transformation TsL = TLW.transform(Ts);
			PriorityQueue<SearchNode> pq = new PriorityQueue<>();
			resultTrajectories = new ArrayList<>();
			pq.add(new SearchNode(null, getUs(), getUs(), TsL, 0.0, -switchCost, 0.0));
			minTime = Double.POSITIVE_INFINITY;
			minDist = Double.POSITIVE_INFINITY;
			Trajectory result = null;
			while (!pq.isEmpty()) {
				SearchNode node = pq.poll();
				for (Control u : U) {
					if (node.next.equals(u))
						continue;
					TransformationTime[] TransformationTimePair= U.completeSegment2(node.T, node.next, u);
					if (Double.isFinite(TransformationTimePair[0].getTime())) {
						Trajectory traj = checkGoal(node, u, controlLine, TransformationTimePair[0], goalMap);
						if (traj != null)
							result = traj;
						else {
							Transformation T = TransformationTimePair[0].getTransformation();
							double duration = TransformationTimePair[0].getTime();
							double cost = node.cost + duration + switchCost;
							double distToGoal = Tf.toPoint().distance(T.toPoint());
							double heuristic = cost + distToGoal / vmax;
							if (heuristic < maxSwitches * switchCost)
								pq.add(new SearchNode(node, node.next, u, T, duration, cost, heuristic));
						}
					}
					if (Double.isFinite(TransformationTimePair[1].getTime())) {
						Trajectory traj = checkGoal(node, u, controlLine, TransformationTimePair[1], goalMap);
						if (traj != null)
							result = traj;
						else {
							Transformation T = TransformationTimePair[1].getTransformation();
							double duration = TransformationTimePair[1].getTime();
							double cost = node.cost + duration + switchCost;
							double distToGoal = Tf.toPoint().distance(T.toPoint());
							double heuristic = cost + distToGoal / vmax;
							if (heuristic < maxSwitches * switchCost)
								pq.add(new SearchNode(node, node.next, u, T, duration, cost, heuristic));
						}
					}
				}
			}
			if (!Double.isFinite(minTime)) {
				resultTrajectories.add(FreeGenericInfo.INFINITY);
			}
			else if (!findAllTrajectories) {
				resultTrajectories.clear();
				resultTrajectories.add(new CostlyGenericInfo(result, controlLine, new DistanceTime(minDist, minTime)));
			}
			return resultTrajectories;
		}

		@Override
		public boolean hasMultipleSolution() {
			return true;
		}
	}
	
	/**
	 * A class for search node
	 * @author yu-hanlyu
	 *
	 */
	private static class SearchNode implements Comparable<SearchNode> {
		private Transformation T;
		private Control u;
		private Control next;
		private SearchNode parent;
		private double duration;
		private double cost;
		private double heuristic;
		
		public SearchNode(SearchNode parent, Control u, Control next, Transformation T, double duration, double cost, double heuristic) {
			this.T = T;
			this.u = u;
			this.next = next;
			this.parent = parent;
			this.duration = duration;
			this.cost = cost;
			this.heuristic = heuristic;
		}

		@Override
		public int compareTo(SearchNode o) {
			if (heuristic < o.heuristic)
				return -1;
			else if (heuristic > o.heuristic)
				return 1;
			return 0;
		}
	}
}