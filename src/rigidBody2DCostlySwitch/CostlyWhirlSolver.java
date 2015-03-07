package rigidBody2DCostlySwitch;

import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.Utility;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import rigidBody2DFreeSwitch.WhirlSolver;

public class CostlyWhirlSolver extends WhirlSolver {
	private double switchCost;
	private double upperBound;
	
	public CostlyWhirlSolver(ControlSet U, Transformation Ts, double switchCost, double upperBound) {
		super(U, Ts);
		this.switchCost = switchCost;
		this.upperBound = upperBound;
	}

	@Override
	protected TrajectoryInfo whirl() {
		return U.controlStream()
				.mapToDouble(u -> u.getOmega())
				.sorted()
				.distinct()
				.mapToObj(omega -> omega)
				.reduce(TrajectoryInfo.INFINITY, 
                (currentMin, omega) -> 
                {  if (omega == 0.0)
                       return currentMin;
                   TrajectoryInfo sol = whirl(omega); 
                       return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2); 
	}
	
	/**
	 * A class for search node
	 * @author yu-hanlyu
	 *
	 */
	protected static class SearchNode implements Comparable<SearchNode> {
		private SearchNode parent;
		private Control beforeU;
		private Control u;
		private Control u1;
		private double length;
		private double cost;
		private double heuristic;
		
		public SearchNode(SearchNode parent, Control u1, Control beforeU, Control u, double length, double cost, double heuristic) {
			this.parent = parent;
			this.beforeU = beforeU;
			this.u = u;
			this.u1 = u1;
			this.length = length;
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
	protected TrajectoryInfo whirl(Control us, Control uf, double p, List<Control> controls) {
		Point2D cs = us.rotationCenter(Ts);
		Point2D cf = uf.rotationCenter(Tf);
		double dsf = cs.distance(cf);
		Map<Control, Double> rkfMap = new HashMap<>();
		double maxRkf = initialMap(uf, dsf, controls, rkfMap);
		
		PriorityQueue<SearchNode> pq = new PriorityQueue<>();
		for (Control u : controls) {
			if (us.equals(u))
				continue;
			double length = us.rotationCenter().distance(u.rotationCenter());
			pq.add(new SearchNode(null, u, us, u, length, switchCost, switchCost));
		}
		TrajectoryInfo result = TrajectoryInfo.INFINITY;
		while (!pq.isEmpty()) {
			SearchNode node = pq.poll();
			if (node.cost > upperBound)
				continue;
			Double rkf = rkfMap.get(node.u);
			if (rkf != null) {
				double length = node.length;
				if (length >= dsf - rkf && length <= dsf + rkf) {
					double cost = node.cost;
					TrajectoryInfo sol = checkGoal(node, us, node.u, uf, dsf, rkf, length, cost);
					if (sol.compareSolution(result) < 0)
						result = sol;
				}
			}
			for (Control u : controls) {
				if (node.u.equals(u))
					continue;
				double length = node.u.rotationCenter().distance(u.rotationCenter()) + node.length;
				double cost = node.cost + switchCost + Utility.timeToLine(node.beforeU, node.u, u);
				rkf = rkfMap.get(u);
				double distToGoal = 0.0;
				if (rkf == null)
					distToGoal = length < dsf - maxRkf ? dsf - maxRkf - length : 0.0;
				else
					distToGoal = length < dsf - rkf ? dsf - rkf - length : 0.0;
				pq.add(new SearchNode(node, node.u1, node.u, u, length, cost, cost + distToGoal));
			}
		}
		return result;
	}
	
	/**
	 * Checking whether reach the goal or not
	 * @param node a search node
	 * @param us the first control
	 * @param beforeUf the second to the last control
	 * @param uf the last control
	 * @param dsf the distance between the first rotation center to the last rotation center
	 * @param rkf the distance between the second to the last rotation center to the last rotation center
	 * @param dsk the distance between the first rotation center to the second to the last rotation enter
	 * @param cost the cost of second controls to the third to the last control
	 * @return
	 */
	private TrajectoryInfo checkGoal(SearchNode node, Control us, Control beforeUf, Control uf, double dsf, double rkf, double dsk, double cost) {
		Point2D cs = us.rotationCenter(Ts);
		Point2D cf = uf.rotationCenter(Tf);
		double theta = Math.acos((dsf * dsf + dsk * dsk - rkf * rkf) / (2.0 * dsf * dsk));
		double direction = Math.atan2(cf.getY() - cs.getY(), cf.getX() - cs.getX());
		Control bbuf = node.parent == null ? us : node.parent.u;
		double[] ta = tsAndTf(us, node.u1, bbuf, beforeUf, uf, direction - theta, dsk);
		double[] tb = tsAndTf(us, node.u1, bbuf, beforeUf, uf, direction + theta, dsk);
		double[] t;
		double dir;
		
		if (ta[0] < tb[0]) {
			t = ta;
			dir = direction - theta;
		} else {
			t = tb;
			dir = direction + theta;
		}
		Trajectory neg = buildSolution(us, beforeUf, uf, ta, dir, node);
		Trajectory pos = buildSolution(us, beforeUf, uf, tb, dir, node);
		if (!isGoal(Ts.move(neg))) {
	    	throw new RuntimeException("Whirl Error");
	    }
		if (!isGoal(Ts.move(pos))) {
	    	throw new RuntimeException("Whirl Error");
	    }
		TrajectoryInfo negative = TrajectoryInfo.createWhirl(neg);
		TrajectoryInfo positive = TrajectoryInfo.createWhirl(pos);
		double time = t[0] + cost + switchCost;
		foundOneSolution(negative);
		foundOneSolution(positive);
		if (time >= minTime)
			return TrajectoryInfo.INFINITY;
		minTime = time;
		return ta[0] < tb[0] ? negative : positive;
	}
	
	/**
	 * Build solution
	 * @param us
	 * @param beforeUf
	 * @param uf
	 * @param t
	 * @param dir
	 * @param node
	 * @return
	 */
	private Trajectory buildSolution(Control us, Control beforeUf, Control uf, double[] t, double dir, SearchNode node) {
		Trajectory sol = new Trajectory();
		sol.addControl(us, t[1]);
		Trajectory temp = new Trajectory();
		for (; node.parent != null; node = node.parent) {
			double time = node.cost - node.parent.cost - switchCost;
			temp.addControl(node.parent.u, time);
		}
		for (int i = temp.size() - 1; i >= 0; --i) {
			sol.addControl(temp.getControl(i), temp.getDuration(i));
		}
		sol.addControl(beforeUf, t[2]);
		sol.addControl(uf, t[3]);
		return sol;
	}
	
	/**
	 * Compute the duration of the first control, the second to the last control, and the last control
	 * @param us the first control
	 * @param u1 the second control
	 * @param bbuf the third to the last control
	 * @param beforeUf the second to the last control
	 * @param uf the last control
	 * @param direction the control direction
	 * @param dsk the distance between the first rotation center to the last rotation center
	 * @return 
	 */
	private double[] tsAndTf(Control us, Control u1, Control bbuf, Control beforeUf, Control uf, double direction, double dsk) {
		// Rotation u1 to the direction
		double ts = Utility.timeToLine(Ts, us, u1, direction);
		Point2D cs = us.rotationCenter(Ts);
		Point2D cf = uf.rotationCenter(Tf);
		// cbeforeUf is the location of the rotation center of beforeUf
		Point2D cbeforeUf = new Point2D.Double(cs.getX() + dsk * Math.cos(direction),
				                               cs.getY() + dsk * Math.sin(direction));
		
		// Compute the direction from cbuf to cf, this is the direction
	    // of rotation center from buf to uf after buf is applied
	    double dirkf = Math.atan2(cf.getY() - cbeforeUf.getY(), cf.getX() - cbeforeUf.getX());
	    // Compute the angle between beforeUf to bbuf and buf to uf
	    // direction + this angle is the direction from cbeforeUf to cf before
	    // beforeUf is applied
	    double diruf = Utility.angle(beforeUf, bbuf, uf) + direction + Math.PI;
	    double tbf = Utility.timeToAngle(diruf, dirkf, beforeUf.getOmega());
	    
	    // Compute the angle from uf to the reference point of robot
	    // before uf is applied
	    double dirf = Utility.angle(uf, beforeUf, new Control(0, 0, 1)) + dirkf + Math.PI;
	    // Compute the angle from uf to the reference point of robot
	    // after uf is applied
	    double dirFinal = Math.atan2(-cf.getY(), -cf.getX());
	    double tf = Utility.timeToAngle(dirf, dirFinal, uf.getOmega());
	    return new double[]{ts + tbf + tf, ts, tbf, tf};
	}
	
	/**
	 * Initialize the rkf map
	 * @param uf the last control
	 * @param dsf
	 * @param controls
	 * @param map
	 * @return
	 */
	private static double initialMap(Control uf, double dsf, List<Control> controls, Map<Control, Double> map) {
		double maxRkf = Double.NEGATIVE_INFINITY;
		for (Control u : controls) {
			if (uf.equals(u))
				continue;
			double rkf = uf.rotationCenter().distance(u.rotationCenter());
			if (!Utility.absEqual(dsf, rkf)) {
				map.put(u, rkf);
				if (rkf > maxRkf)
					rkf = maxRkf;
			}
		}
		return maxRkf;
	}
}
