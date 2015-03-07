package rigidBody2DCostlySwitch;

import java.util.PriorityQueue;

import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.TransformationTime;
import optimalControl.Utility;
import rigidBody2DFreeSwitch.TGTSolver;
import rigidBody2DFreeSwitch.TrajectoryInfo;

public class CostlyTGTSolver extends TGTSolver {
	private double switchCost;
	private double vmax;
	private double upperBound;
	
	public CostlyTGTSolver(ControlSet U, Transformation Ts, double switchCost, double upperBound) {
		super(U, Ts);
		vmax = U.maxVelocity();
		this.switchCost = switchCost;
		this.upperBound = upperBound;
	}
	
	/**
	 * A class for search node
	 * @author yu-hanlyu
	 *
	 */
	protected static class SearchNode implements Comparable<SearchNode> {
		private SearchNode parent;
		private Transformation afterUs;
		private Control u;
		private Control next;
		private Transformation T;
		private double duration;
		private double cost;
		private double heuristic;
		
		public SearchNode(SearchNode parent, Transformation afterUs, Control u, Control next, Transformation T, double duration, double cost, double heuristic) {
			this.parent = parent;
			this.afterUs = afterUs;
			this.u = u;
			this.next = next;
			this.T = T;
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

	@Override
	protected TrajectoryInfo TGT(Control us, Control uf, ControlLine controlLine) {
		Transformation TLW = new Transformation(controlLine);
		Transformation TsL = TLW.transform(Ts);
		
		// Initialize the priority queue
		PriorityQueue<SearchNode> pq = new PriorityQueue<>();
		for (Control u : U) {
			if (us.equals(u))
				continue;
			TransformationTime[] TransformationTimePair= U.completeSegment2(TsL, us, u);
			if (Double.isFinite(TransformationTimePair[0].getTime())) {
				Transformation T = TransformationTimePair[0].getTransformation();
				double duration = TransformationTimePair[0].getTime();
				double cost = duration + switchCost;
				double distToGoal = Tf.toPoint().distance(T.toPoint());
				double heuristic = cost + distToGoal / vmax;
				pq.add(new SearchNode(null, T, us, u, T, duration, cost, heuristic));
			}
			if (Double.isFinite(TransformationTimePair[1].getTime())) {
				Transformation T = TransformationTimePair[1].getTransformation();
				double duration = TransformationTimePair[1].getTime();
				double cost = duration + switchCost;
				double distToGoal = Tf.toPoint().distance(T.toPoint());
				double heuristic = cost + distToGoal / vmax;
				pq.add(new SearchNode(null, T, us, u, T, duration, cost, heuristic));
			}
		}
		
		TrajectoryInfo result = TrajectoryInfo.INFINITY;
		while (!pq.isEmpty()) {
			SearchNode node = pq.poll();
			if (uf.equals(node.next)) {
				TrajectoryInfo sol = checkGoal(node, us, uf, controlLine);
				if (sol.compareSolution(result) < 0)
					result = sol;
			}
			for (Control u : U) {
				if (u.equals(node.next))
					continue;
				TransformationTime[] TransformationTimePair= U.completeSegment2(node.T, node.next, u);
				if (Double.isFinite(TransformationTimePair[0].getTime())) {
					Transformation T = TransformationTimePair[0].getTransformation();
					double duration = TransformationTimePair[0].getTime();
					double cost = node.cost + duration + switchCost;
					double distToGoal = Tf.toPoint().distance(T.toPoint());
					double heuristic = cost + distToGoal / vmax;
					if (heuristic < upperBound)
						pq.add(new SearchNode(node, node.afterUs, node.next, u, T, duration, cost, heuristic));
				}
				if (Double.isFinite(TransformationTimePair[1].getTime())) {
					Transformation T = TransformationTimePair[1].getTransformation();
					double duration = TransformationTimePair[1].getTime();
					double cost = node.cost + duration + switchCost;
					double distToGoal = Tf.toPoint().distance(T.toPoint());
					double heuristic = cost + distToGoal / vmax;
					if (heuristic < upperBound)
						pq.add(new SearchNode(node, node.afterUs, node.next, u, T, duration, cost, heuristic));
				}
			}
		}
		return result;
	}
	
	/**
	 * Check goal
	 * @param node
	 * @param us
	 * @param uf
	 * @param controlLine
	 * @return
	 */
	private TrajectoryInfo checkGoal(SearchNode node, Control us, Control uf, ControlLine controlLine) {
		if (!uf.equals(node.next))
			return TrajectoryInfo.INFINITY;
		Transformation beforeUf = node.T;
		Transformation TLW = new Transformation(controlLine);
		Transformation TfL = TLW.transform(Tf);
		Transformation TsL = TLW.transform(Ts);
		if (!Utility.angleEqual(beforeUf.getTheta(), TfL.getTheta()))
			return TrajectoryInfo.INFINITY;
		Control usL = us.toWorld(TsL);
	    Control ufL = uf.toWorld(TfL);
		double[] t0AndTf = tsAndTf(TsL, node.afterUs, beforeUf, TfL, usL, ufL);
		double ts = t0AndTf[0];
	    double tf = t0AndTf[1];
	    if (!Double.isFinite(ts))
	    	return TrajectoryInfo.INFINITY;
	    
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(us, ts);
	    Trajectory temp = new Trajectory();
		for ( ; node.parent != null; node = node.parent) {
			temp.addControl(node.u, node.duration);
		}
		for (int i = temp.size() - 1; i >= 0; --i) {
			trajectory.addControl(temp.getControl(i), temp.getDuration(i));
		}
		trajectory.addControl(uf, tf);
		if (!isGoal(Ts.move(trajectory))) {
	    	throw new RuntimeException("TGT Error");
	    }
		TrajectoryInfo info = TrajectoryInfo.createTGT(trajectory, controlLine);
	    foundOneSolution(info);
	    return info;
	}
}
