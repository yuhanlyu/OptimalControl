package rigidBody2DFreeSwitch;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.stream.IntStream;

import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlLineFactory;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.Utility;

/**
 * Solver for singular trajectories
 * @author yu-hanlyu
 *
 */
public class SingularSolver extends OptimalTrajectorySolver {
	protected static final double MIN_SINGULAR_DURATION = 0.00001; // for Omni-directional vehicle
	protected double upperBound;

	/**
	 * Constructor
	 * @param U
	 * @param Ts
	 * @param upperBound
	 */
	public SingularSolver(ControlSet U, Transformation Ts, double upperBound) {
		super(U, Ts);
		this.upperBound = upperBound;
	}

	@Override
	public TrajectoryInfo solve() {
		solution = singular();
		minTime = solution.getTime();
		return solution;
	}

	/**
	 * Find optimal singular trajectories
	 * @return
	 */
	private TrajectoryInfo singular() {
		return U.controlStream().reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, us) -> 
                                        {  TrajectoryInfo sol = singular(us); 
                                           return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2); 
	}
	
	/**
	 * Find optimal singular trajectories with first control us
	 * @param us
	 * @return
	 */
	private TrajectoryInfo singular(Control us) {
		return U.controlStream().filter(uf -> us.isRotation() || uf.isRotation())
				                .reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, uf) -> 
                                        {  TrajectoryInfo sol = singular(us, uf); 
                                           return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2);
	}
	
	/**
	 * Find optimal singular trajectories with first control us and the last control uf
	 * @param us
	 * @param uf
	 * @return
	 */
	private TrajectoryInfo singular(Control us, Control uf) {
		ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, true);
		TrajectoryInfo positive = singular(us, uf, factory);
		factory = new ControlLineFactory(U, Ts, us, Tf, uf, false);
		TrajectoryInfo negative = singular(us, uf, factory);
		return positive.compareSolution(negative) < 0 ? positive : negative;
	}
	
	/**
	 * Find optimal singular trajectories with first control us, the last control uf, and sign isPositive
	 * @param us
	 * @param uf
	 * @param isPositive
	 * @param factory
	 * @return
	 */
	private TrajectoryInfo singular(Control us, Control uf, ControlLineFactory factory) {
		return IntStream.range(0, U.getCriticalSize())
		                .boxed()
		                .reduce(TrajectoryInfo.INFINITY, 
                                 (currentMin, i) -> 
                                 {  double H = U.getCriticalValue(i);
                                    ControlLine controlLine = factory.getControlLine(H, true);
                                    TrajectoryInfo sol = singular(us, uf, controlLine); 
                                    return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                   (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2);
	}
	
	/**
	 * Find optimal singular trajectories with first control us, the last control uf, and control line
	 * @param us
	 * @param uf
	 * @param controlLine
	 * @return
	 */
	protected TrajectoryInfo singular(Control us, Control uf, ControlLine controlLine) {
		if (!controlLine.isValid())
			return TrajectoryInfo.INFINITY;
	    Transformation TLW = new Transformation(controlLine);
	    Transformation TsL = TLW.transform(Ts);
	    Transformation TfL = TLW.transform(Tf);
	    // It is possible that us and uf are not maximizing controls
	    if (!U.isMaximizing(TsL, us) || !U.isMaximizing(TfL, uf)) {
	    	return TrajectoryInfo.INFINITY;
	    }
	    
	    Trajectory ga = new Trajectory();
	    Trajectory gb = new Trajectory();
	    Transformation beforeMid = typeAsingular(TsL, us, ga);
	    Transformation afterMid  = typeBsingular(TfL, uf, gb);
	    
	    if (beforeMid == null || afterMid == null)
	    	return TrajectoryInfo.INFINITY;
	    
	    List<Trajectory> midTrajectories = new ArrayList<>();
	    Trajectory mid = allSingulars(beforeMid, afterMid, controlLine.getH(), midTrajectories);
	    if (mid == null)
	    	return TrajectoryInfo.INFINITY;
	    
	    for (Trajectory traj : midTrajectories) {
	    	Trajectory trajectory = new Trajectory();
	    	trajectory.append(ga);
	    	trajectory.append(traj);
	    	trajectory.append(gb);
	    	foundOneSolution(TrajectoryInfo.createSingular(trajectory, controlLine));
	    	if (!isGoal(Ts.move(trajectory))) {
		    	System.out.println("Singular Error: " + us + " " + uf + " " + controlLine);
		    	double error = Tf.toPoint().distance(Ts.move(trajectory).toPoint());
		    	System.out.println("Error is " + error);
		    	if (error > Utility.EPSILON * 10)
		    		throw new RuntimeException("Singular Error");
		    }
	    }
	    Trajectory trajectory = new Trajectory();
	    trajectory.append(ga);
	    trajectory.append(mid);
	    trajectory.append(gb);
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
	 * Find a trajectory from TLR to the control line 
	 * assuming the first control is us
	 * @param TLR
	 * @param us
	 * @param trajectory an empty trajectory
	 * @return
	 */
	protected Transformation typeAsingular(Transformation TLR, Control us, Trajectory trajectory) {
		List<Control> sustainableControls = U.sustainableControls(TLR);
		if (!sustainableControls.contains(us))
			return null;
		Trajectory temp = U.getExcursion(TLR, MIN_SINGULAR_DURATION);
		if (temp == null)
			return null;
		trajectory.append(temp);
		return TLR.move(trajectory);
	}
	
	/**
	 * Find a trajectory backward from TLR to the control line, 
     * assuming the last control is uf
	 * @param TLR
	 * @param uf
	 * @param trajectory must be an empty trajectory
	 *        it will be modified to contain the trajectory from TLR to the return value
	 *        if return value is null, then trajectory can be arbitrary
	 * @return 
	 */
	protected Transformation typeBsingular(Transformation TLR, Control uf, Trajectory trajectory) {
		ControlSet Ur = U.reverse();
	    Transformation reflectedTLR = TLR.reflect();
	    List<Control> sustainableControls = Ur.sustainableControls(reflectedTLR);
		if (!sustainableControls.contains(uf.reverse()))
			return null;
		Trajectory temp = Ur.getExcursion(reflectedTLR, MIN_SINGULAR_DURATION);
		if (temp == null)
			return null;
		for (int i = temp.size() - 1; i >= 0; --i)
			trajectory.addControl(temp.getControl(i).reverse(), temp.getDuration(i));
		return reflectedTLR.move(temp).reflect();
	}
	
	/**
	 * Search a fastest trajectory from Ta to Tb, where both are 
     * on the control line
	 * @param TLa starting configuration, multiple sustainable controls
	 * @param TLb final configuration, multiple sustainable controls
	 * @param H the Hamiltonian value
	 * @return a trajectory or null if a trajectory does not exist
	 */
	protected Trajectory allSingulars(Transformation TLa, Transformation TLb, double H, List<Trajectory> midTrajectories) {
		List<SearchNode> forward = generateForward(TLa);
		List<SearchNode> backward = generateBackward(TLb);
		Trajectory minSolution = checkGoal(forward, backward, H, midTrajectories);
		return minSolution;
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
				if (sol != null && sol.totalTime() < minSolution.totalTime())
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
	
	/**
	 * Generate all search nodes backward from TLa
	 * @param TLa a configuration in the control line frame
	 * @return a list of search nodes containing all search nodes
	 */
	private List<SearchNode> generateForward(Transformation TLa) {
		PriorityQueue<SearchNode> pq = new PriorityQueue<>();
		List<SearchNode> result = new ArrayList<>();
		SearchNode root = new SearchNode(TLa, null, new Trajectory(), 0.0);
		pq.add(root);
		result.add(root);
		while (!pq.isEmpty()) {
			SearchNode node = pq.poll();
			if (node.getCost() > upperBound) {
				continue;
			}
			List<Control> sustainableControls = U.sustainableControls(node.getTransformation());
			for (Control u : sustainableControls) {
				if (u.isTranslation())
					continue;
				Trajectory trajectory = U.getExcursion(node.getTransformation(), u, MIN_SINGULAR_DURATION);
				if (trajectory == null)
					continue;
				Transformation newT = node.getTransformation().move(trajectory);
				SearchNode newNode = new SearchNode(newT, node, trajectory, trajectory.totalTime() + node.getCost());
				pq.add(newNode);
				result.add(newNode);
			}
		}
		return result;
	}
	
	/**
	 * Generate all search nodes backward from TLb
	 * @param TLb a configuration in the control line frame
	 * @return a list of search nodes containing all search nodes
	 */
	private List<SearchNode> generateBackward(Transformation TLb) {
		ControlSet Ur = U.reverse();
		PriorityQueue<SearchNode> pq = new PriorityQueue<>();
		List<SearchNode> result = new ArrayList<>();
		SearchNode root = new SearchNode(TLb, null, new Trajectory(), 0.0);
		pq.add(root);
		result.add(root);
		
		while (!pq.isEmpty()) {
			SearchNode node = pq.poll();
			if (node.getCost() > upperBound) {
				continue;
			}
			Transformation reflectedT = node.getTransformation().reflect();
			List<Control> sustainableControls = Ur.sustainableControls(reflectedT);
			for (Control u : sustainableControls) {
				if (u.isTranslation())
					continue;
				Trajectory trajectory = Ur.getExcursion(reflectedT, u, MIN_SINGULAR_DURATION);
				if (trajectory == null)
					continue;
				Transformation newT = reflectedT.move(trajectory).reflect();
				SearchNode newNode = new SearchNode(newT, node, trajectory.reflect(), trajectory.totalTime() + node.getCost());
				pq.add(newNode);
				result.add(newNode);
				
			}
		}
		return result;
	}
	
	protected void foundOneSolution(TrajectoryInfo createSingular) {
	}
	
	/**
	 * Helper class for search
	 * @author yu-hanlyu
	 *
	 */
	private static class SearchNode implements Comparable<SearchNode> {
		private Transformation T;
		private SearchNode parent;
		private Trajectory trajectory;
		private double cost;
		
		/**
		 * Constructor
		 * @param T
		 * @param parent
		 * @param trajectory
		 * @param cost
		 */
		public SearchNode(Transformation T, SearchNode parent, Trajectory trajectory, double cost) {
			this.T = T;
			this.parent = parent;
			this.trajectory = trajectory;
			this.cost = cost;
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
				result.append(path.get(i).trajectory);
			}
			return result;
		}
		
		@Override
		public int compareTo(SearchNode o) {
			if (cost < o.cost)
				return -1;
			else if (cost > o.cost)
				return 1;
			return 0;
		}
		
		/**
		 * Return the cost
		 * @return
		 */
		public double getCost() {
			return cost;
		}
		
		/**
		 * Return the transformation
		 * @return
		 */
		public Transformation getTransformation() {
			return T;
		}
	}
}
