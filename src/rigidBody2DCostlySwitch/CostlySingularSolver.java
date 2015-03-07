package rigidBody2DCostlySwitch;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.TransformationTime;
import optimalControl.Utility;
import rigidBody2DFreeSwitch.SingularSolver;
import rigidBody2DFreeSwitch.TrajectoryInfo;

public class CostlySingularSolver extends SingularSolver {
	private double switchCost;
	private double vmax;
	
	public CostlySingularSolver(ControlSet U, Transformation Ts,
			double switchCost, double upperBound) {
		super(U, Ts, upperBound);
		this.switchCost = switchCost;
		this.vmax = U.maxVelocity();
	}

	/**
	 * Find optimal singular trajectories with first control us, the last control uf, and control line
	 * @param us
	 * @param uf
	 * @param controlLine
	 * @return
	 */
	@Override
	protected TrajectoryInfo singular(Control us, Control uf, ControlLine controlLine) {
		if (!controlLine.isValid())
			return TrajectoryInfo.INFINITY;
	    Transformation TLW = new Transformation(controlLine);
	    Transformation TsL = TLW.transform(Ts);
	    Transformation TfL = TLW.transform(Tf);
	    
	    List<Trajectory> midTrajectories = new ArrayList<>();
	    Trajectory trajectory = allSingulars(TsL, TfL, controlLine.getH(), midTrajectories);
	    if (trajectory == null)
	    	return TrajectoryInfo.INFINITY;
	    
	    for (Trajectory traj : midTrajectories) {
	    	foundOneSolution(TrajectoryInfo.createSingular(traj, controlLine));
	    	if (!isGoal(Ts.move(traj))) {
		    	System.out.println("Singular Error: " + us + " " + uf + " " + controlLine);
		    	double error = Tf.toPoint().distance(Ts.move(traj).toPoint());
		    	System.out.println("Error is " + error);
		    	if (error > Utility.EPSILON * 10)
		    		throw new RuntimeException("Singular Error");
		    }
	    }
	    if (!isGoal(Ts.move(trajectory))) {
	    	System.out.println("Singular Error: " + us + " " + uf + " " + controlLine);
	    	double error = Tf.toPoint().distance(Ts.move(trajectory).toPoint());
	    	System.out.println("Error is " + error);
	    	if (error > Utility.EPSILON * 10)
	    		throw new RuntimeException("Singular Error");
	    }
	    return TrajectoryInfo.createSingular(trajectory, controlLine);
	}
	
	/**
	 * Search a fastest trajectory from Ta to Tb, where both are 
     * on the control line
	 * @param TLa starting configuration, multiple sustainable controls
	 * @param TLb final configuration, multiple sustainable controls
	 * @param H the Hamiltonian value
	 * @return a trajectory or null if a trajectory does not exist
	 */
	protected Trajectory allSingulars(Transformation TsL, Control us, Transformation TfL, Control uf, double H, List<Trajectory> midTrajectories) {
		List<SearchNode> forward = generateForward(TsL, us);
		List<SearchNode> backward = generateBackward(TfL, uf);
		Trajectory minSolution = checkGoal(forward, backward, H, midTrajectories);
		return minSolution;
	}

	/**
	 * Generate all search nodes backward from TLa
	 * @param TsL a configuration in the control line frame
	 * @return a list of search nodes containing all search nodes
	 */
	private List<SearchNode> generateForward(Transformation TsL, Control us) {
		PriorityQueue<SearchNode> pq = new PriorityQueue<>();
		List<SearchNode> result = new ArrayList<>();
		SearchNode root = new SearchNode(null, us, us, TsL, 0.0, -switchCost, 0.0);
		pq.add(root);
		result.add(root);
		while (!pq.isEmpty()) {
			SearchNode node = pq.poll();
			for (Control u : U) {
				if (u.isTranslation() || node.next.equals(u))
					continue;
				TransformationTime[] TransformationTimePair= U.completeSegment2(node.T, node.next, u, true);
				if (Double.isFinite(TransformationTimePair[0].getTime())) {
					Transformation T = TransformationTimePair[0].getTransformation();
					double duration = TransformationTimePair[0].getTime();
					double cost = node.cost + duration + switchCost;
					double distToGoal = Tf.toPoint().distance(T.toPoint());
					double heuristic = cost + distToGoal / vmax;
					if (heuristic < upperBound) {
						SearchNode newNode = new SearchNode(node, node.next, u, T, duration, cost, heuristic);
						pq.add(newNode);
						result.add(newNode);
					}
				}
				if (Double.isFinite(TransformationTimePair[1].getTime())) {
					Transformation T = TransformationTimePair[1].getTransformation();
					double duration = TransformationTimePair[1].getTime();
					double cost = node.cost + duration + switchCost;
					double distToGoal = Tf.toPoint().distance(T.toPoint());
					double heuristic = cost + distToGoal / vmax;
					if (heuristic < upperBound) {
						SearchNode newNode = new SearchNode(node, node.next, u, T, duration, cost, heuristic);
						pq.add(newNode);
						result.add(newNode);
					}
				}
			}
		}
		return result;
	}
	
	/**
	 * Generate all search nodes backward from TLb
	 * @param TfL a configuration in the control line frame
	 * @return a list of search nodes containing all search nodes
	 */
	private List<SearchNode> generateBackward(Transformation TfL, Control uf) {
		ControlSet Ur = U.reverse();
		PriorityQueue<SearchNode> pq = new PriorityQueue<>();
		List<SearchNode> result = new ArrayList<>();
		SearchNode root = new SearchNode(null, uf, uf, TfL, 0.0, -switchCost, 0.0);
		pq.add(root);
		result.add(root);
		while (!pq.isEmpty()) {
			SearchNode node = pq.poll();
			for (Control u : U) {
				if (node.next.equals(u))
					continue;
				TransformationTime[] TransformationTimePair= U.completeSegment2(node.T.reflect(), node.next.reverse(), u.reverse(), true);
				if (Double.isFinite(TransformationTimePair[0].getTime())) {
					Transformation T = TransformationTimePair[0].getTransformation().reflect();
					double duration = TransformationTimePair[0].getTime();
					double cost = node.cost + duration + switchCost;
					double distToGoal = Tf.toPoint().distance(T.toPoint());
					double heuristic = cost + distToGoal / vmax;
					if (heuristic < upperBound) {
						SearchNode newNode = new SearchNode(node, node.next, u, T, duration, cost, heuristic);
						pq.add(newNode);
						result.add(newNode);
					}
				}
				if (Double.isFinite(TransformationTimePair[1].getTime())) {
					Transformation T = TransformationTimePair[1].getTransformation().reflect();
					double duration = TransformationTimePair[1].getTime();
					double cost = node.cost + duration + switchCost;
					double distToGoal = Tf.toPoint().distance(T.toPoint());
					double heuristic = cost + distToGoal / vmax;
					if (heuristic < upperBound) {
						SearchNode newNode = new SearchNode(node, node.next, u, T, duration, cost, heuristic);
						pq.add(newNode);
						result.add(newNode);
					}
				}
			}
		}
		return result;
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
		
		/**
		 * Construct a trajectory follow by parent pointer
		 * @param forward if forward is true, construct the trajectory in the reverse order 
		 * @return
		 */
		public Trajectory chain(boolean forward) {
			SearchNode node = this;
			Trajectory result = new Trajectory();
			List<SearchNode> path = new LinkedList<>();
			for (; node != null; node = node.parent) {
				if (forward) {
					path.add(0, node);
				} else
					path.add(node);
			}
			for (int i = 0; i < path.size(); ++i) {
				result.addControl(path.get(i).u, path.get(i).duration);
			}
			return result;
		}
	}
	
	/**
	 * Find the fastest trajectory from any node in forward to any node in backward
	 * @param forward a list of node
	 * @param backward a list of node
	 * @param H the Hamiltonian value
	 * @param midTrajectories storing all possible trajectories
	 * @return the fastest trajectory or null if no trajectory exists
	 */
	private Trajectory checkGoal(List<SearchNode> forward, List<SearchNode> backward, double H, List<Trajectory> midTrajectories) {
		Trajectory minSolution = new Trajectory();
		minSolution.addControl(U.getControl(0), Double.POSITIVE_INFINITY);
		
		for (SearchNode forwardNode : forward) {
			for (SearchNode backwardNode : backward) {
				Trajectory sol = checkGoal(forwardNode, backwardNode, H);
				if (sol != null) {
					midTrajectories.add(sol);
				}
				if (sol != null && sol.getCost(switchCost) < minSolution.getCost(switchCost))
					minSolution = sol;
			}
		}
		return Double.isInfinite(minSolution.totalTime()) ? null : minSolution;
	}
	
	/**
	 * Check whether it is possible from orward to backward by a translation
	 * @param forward a searchnode
	 * @param backward a searchnode
	 * @param H the Hamiltonian value
	 * @return a trajectory or null if no trajectory exists
	 */
	private Trajectory checkGoal(SearchNode forward, SearchNode backward, double H) {
		Transformation before = forward.T;
		Transformation after = backward.T;
		if (!Utility.absEqual(before.getY(), after.getY()) || !Utility.angleEqual(before.getTheta(), after.getTheta()))
			return null;
		double dist = after.getX() - before.getX();
	    if (dist < 0.0) 
	    	return null;
	    
	    // Checking whether there exists a translation
	    Control u = U.singularTranslation(before, H);
	    if (u == null)
	    	return null;
	    Trajectory forwardTrajectory = forward.chain(true);
	    Trajectory backwardTrajectory = backward.chain(false);
	    double t = dist / H;
	    forwardTrajectory.addControl(u, t);
	    forwardTrajectory.append(backwardTrajectory);
	    return forwardTrajectory;
	}
}
