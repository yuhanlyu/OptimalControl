package rigidBody2DFreeSwitch;
import java.awt.geom.Point2D;
import java.util.stream.IntStream;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.Utility;

/**
 * Solver for whirl trajectories
 * @author yu-hanlyu
 *
 */
public class WhirlSolver extends OptimalTrajectorySolver {
	
	public WhirlSolver(ControlSet U, Transformation Ts) {
		super(U, Ts);
	}

	@Override
	public TrajectoryInfo solve() {
		minTime = Double.POSITIVE_INFINITY;
		solution = whirl();
		minTime = solution.getTime();
		return solution;
	}
	
	/**
	 * If all controls have the same angular speed with maximum magnitude
	 * @return
	 */
	protected TrajectoryInfo whirl() {
		double minOmega = U.controlStream().mapToDouble(u -> u.getOmega()).min().getAsDouble();
		double maxOmega = U.controlStream().mapToDouble(u -> u.getOmega()).max().getAsDouble();
		TrajectoryInfo max = whirl(maxOmega);
		TrajectoryInfo min = whirl(minOmega);
		return min.compareSolution(max) < 0 ? min : max;
	}
	
	/**
	 * Find whirl trajectory for a fixed angular velocity
	 * @param omega
	 * @return
	 */
	protected TrajectoryInfo whirl(double omega) {
		List<Control> controls = U.controlStream().filter(u -> Utility.absEqual(u.getOmega(), omega))
				                                  .collect(Collectors.toList());
		return whirl(convexHull(controls));
	}	
	
	/**
	 * Find whirl trajectory for a fixed sequence of controls
	 * @param controls
	 * @return
	 */
	private TrajectoryInfo whirl(List<Control> controls) {
	    if (controls.size() < 2)
	        return TrajectoryInfo.INFINITY;
	    double p = IntStream.range(0, controls.size()).mapToDouble(i -> controls.get(i).distance(controls.get((i+1) % controls.size()))).sum();
	    return controls.stream().reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, us) -> 
                                        {  TrajectoryInfo sol = whirl(us, p, controls); 
                                           return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2);
	}
	
	/**
	 * Find whirl trajectory for a fixed sequence of controls with first control us
	 * @param us the first control
	 * @param p
	 * @param controls
	 * @return
	 */
	private TrajectoryInfo whirl(Control us, double p, List<Control> controls) {
		return controls.stream().reduce(TrajectoryInfo.INFINITY, 
                (currentMin, uf) -> 
                {  TrajectoryInfo sol = whirl(us, uf, p, controls); 
                   return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2);
	}
	
	/**
	 * Find whirl trajectory for given first control and the last control
	 * @param us
	 * @param uf
	 * @param p
	 * @param controls
	 * @return
	 */
	protected TrajectoryInfo whirl(Control us, Control uf, double p, List<Control> controls) {
		return controls.stream().filter(beforeUf -> !uf.equals(beforeUf))
				                .reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, beforeUf) -> 
                                        {  TrajectoryInfo sol = whirl(us, beforeUf, uf, p, controls); 
                                            return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2);
	}
	
	/**
	 * Find whirl trajectory for given first control, and last two controls
	 * @param us
	 * @param beforeUf
	 * @param uf
	 * @param p
	 * @param controls
	 * @return
	 */
	private TrajectoryInfo whirl(Control us, Control beforeUf, Control uf, double p, List<Control> controls) {
		//Rotate us to the beginning		
		int middle = controls.indexOf(us);
		Collections.rotate(controls, -middle);
	    
		Point2D cs = us.rotationCenter(Ts);
		Point2D cf = uf.rotationCenter(Tf);
	    
	    double dsf = cs.distance(cf);
	    double rkf = beforeUf.distance(uf);
	    if (Utility.absEqual(dsf, rkf) && us.equals(beforeUf)) {
	    	TrajectoryInfo result = whirlTwo(us, uf);
	    	foundOneSolution(result);
	        return result;
	    }
	    /**
	     * dsf is the length between the first rotation center 
	     * and the last rotation center
	     * d1k is the length between the first rotation center and the second to 
	     * the last rotation center
	     * the distance between the last two rotations is rkf
	     * dsf, rkf can be computed easily, we just need to determine d1k
	     */
	    double lk = 0.0;
	    for (int i = 0; i < controls.size(); ++i) {
	        if (controls.get(i).equals(beforeUf))
	            break;
	        lk += controls.get(i).distance(controls.get((i+1) % controls.size()));
	    }

	    double n = Math.floor((dsf + rkf - lk) / p);
	    double d1k = n * p + lk;
	    double direction = Math.atan2(cf.getY() - cs.getY(), cf.getX() - cs.getX());
	    double theta = Math.acos((dsf * dsf + d1k * d1k - rkf * rkf) 
                                  / (2.0 * dsf * d1k));
	    if (Double.isNaN(theta))
	        return TrajectoryInfo.INFINITY;
	    TrajectoryInfo positive = whirl(us, beforeUf, uf, (int)n, direction + theta, controls);
	    TrajectoryInfo negative = whirl(us, beforeUf, uf, (int)n, direction - theta, controls);
	    foundOneSolution(positive);
	    foundOneSolution(negative);
	    return positive.compareSolution(negative) < 0 ? positive : negative;
	}
	
	
	/**
	 * Compute whirl trajectory for a set of controls with the same angular speed
     * with the first control u0 and the last control uf and the second to
     * the last control before_uf
     * the control line direction is dir and the number of cycles is n
	 * @param us
	 * @param beforeUf
	 * @param uf
	 * @param n
	 * @param direction
	 * @param controls
	 * @return
	 */
	private TrajectoryInfo whirl(Control us, Control beforeUf, Control uf, int n, double direction, List<Control> controls) {
		Trajectory trajectory = new Trajectory();
	    double ta = Utility.timeToLine(Ts, us, controls.get(1), direction);
	    Transformation T = Ts.move(us, ta);
	    trajectory.addControl(us, ta);
	    
	    int index = 1; // Will be the index of before_uf in controls
	    // Finish the partial cycle until encounter before_uf 
	    if (!controls.get(1).equals(beforeUf)) {
	        for (int i = 1; i < controls.size(); ++i) {
	        	Control u = controls.get(i);
	        	Control next = controls.get((i+1) % controls.size());
	            double t = Utility.timeToLine(T, u, next, direction);
	            T = T.move(u, t);
	            trajectory.addControl(u, t);
	            if (next.equals(beforeUf)) {
	                index = (i + 1) % controls.size();
	                break;
	            }
	        }
	        if (controls.get(0).equals(beforeUf)) {
	            --n;
	            index = 0;
	        }
	    }

	    // Finish all cycles
	    for (; n > 0; --n) {
	        for (int i = 0; i < controls.size(); ++i) {
	            Control u = controls.get((i+index) % controls.size());
	            Control next = controls.get((i+index+1) % controls.size());
	            double t = Utility.timeToLine(T, u, next, direction);
	            T = T.move(u, t);
	            trajectory.addControl(u, t);
	        }
	    }

	    // Finish the second to the last control
	    Point2D ck = beforeUf.rotationCenter(T);
	    Point2D cf = uf.rotationCenter(Tf);

	    double dir = Math.atan2(cf.getY() - ck.getY(), cf.getX() - ck.getX());
	    double tk = Utility.timeToLine(T, beforeUf, uf, dir);
	    T = T.move(beforeUf, tk);
	    trajectory.addControl(beforeUf, tk);

	    // Finish the last control
	    double tf = Utility.timeToAngle(T.getTheta(), Tf.getTheta(), uf.getOmega());
	    T = T.move(uf, tf);
	    trajectory.addControl(uf, tf);
	    if (!isGoal(Ts.move(trajectory))) {
	    	throw new RuntimeException("Whirl Error");
	    }
	    return TrajectoryInfo.createWhirl(trajectory);
	}
	
	/**
	 * Compute whirl trajectory for only two actions us and uf
	 * @param us
	 * @param uf
	 * @return
	 */
	private TrajectoryInfo whirlTwo(Control us, Control uf) {
		Point2D cs = us.rotationCenter(Ts);
		Point2D cf = uf.rotationCenter(Tf);
	    double dir = Math.atan2(cf.getY() - cs.getY(), cf.getX() - cs.getX());
	    double ts = Utility.timeToLine(Ts, us, uf, dir);
	    Transformation beforeUf = Ts.move(us, ts);
	    double tf = Utility.timeToAngle(beforeUf.getTheta(), Tf.getTheta(), uf.getOmega());
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(us, ts);
	    trajectory.addControl(uf, tf);
	    if (!isGoal(Ts.move(trajectory))) {
	    	throw new RuntimeException("Whirl Error");
	    }
	    return TrajectoryInfo.createWhirl(trajectory);
	}
	
	/**
	 * Compute the convex hull in counter clockwise order of controls
     * All controls have the same angular velocity and they are
     * vertices of the convex hull. Hence, we only need to sort them.
	 * @param controls
	 * @return
	 */
	private static List<Control> convexHull(List<Control> controls) {
		Collections.sort(controls, (u1, u2) -> {
			double angle1 = Utility.normalize(Math.atan2(u1.getVy(), u1.getVx()));
			double angle2 = Utility.normalize(Math.atan2(u2.getVy(), u2.getVx()));
			return angle1 < angle2 ? -1 : 1;
		});
		return controls;
	}
	
	/**
	 * Invoked when found one trajectory
	 * @param trajectory
	 */
	protected void foundOneSolution(TrajectoryInfo trajectory) {
	}
}
