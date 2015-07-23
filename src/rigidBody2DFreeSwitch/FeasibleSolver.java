package rigidBody2DFreeSwitch;
import java.awt.geom.Point2D;
import java.util.logging.Logger;

import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.Homogeneous;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.Utility;

/**
 * 
 * @author yu-hanlyu
 *
 */
public class FeasibleSolver extends OptimalTrajectorySolver {
	
	private static final Logger logger = Logger.getLogger(FeasibleSolver.class.getName());

	/**
	 * Constructor
	 * @param U
	 * @param Ts
	 */
	public FeasibleSolver(ControlSet U, Transformation Ts) {
		super(U, Ts);
	}

	@Override
	public TrajectoryInfo solve() {
		logger.info("Start to find a feasible solution for configuration " + Ts);
		if (isGoal(Ts)) {
			Trajectory trajectory = new Trajectory();
			trajectory.addControl(U.getControl(0), 0);
			solution = new TrajectoryInfo(trajectory);
	        minTime = 0.0;
	        return solution;
	    }
	    TrajectoryInfo single = singlePlan();
	    TrajectoryInfo pair = pairPlan();
	    solution = single.compareSolution(pair) < 0 ? single : pair;
	    minTime = solution.getTime();
	    return solution;
	}

	/**
	 * Find a possible trajectory using one control
	 * @return a trajectory
	 */
	private TrajectoryInfo singlePlan() {
		logger.fine("In single plan");
		return U.controlStream().reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, u) -> 
                                        {  TrajectoryInfo sol = singlePlan(u); 
                                           return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2);        
	}
	
	/**
	 * Find a possible trajectory using one control u
	 * @param u only control
	 * @return a trajectory
	 */
	private TrajectoryInfo singlePlan(Control u) {
		logger.finer("In single plan " + u);
		return u.isTranslation() ? singleTranslation(u) : singleRotation(u);
	}
	
	/**
	 * Find a possible trajectory using one translation
	 * @param u a translation control
	 * @return a trajectory
	 */
	private TrajectoryInfo singleTranslation(Control u) {
		logger.finest("In single plan with one translation " + u);
	    Control usW = u.toWorld(Ts);
	    Control ufW = u.toWorld(Tf);

	    // If the velocity are not the same, then
	    // we can not reach the goal by only using u
	    if (!usW.close(ufW))
	        return TrajectoryInfo.INFINITY;
	    double time = (Tf.getX() - Ts.getX()) / usW.getVx();
	    if (time < 0.0 || !Utility.absEqual(usW.getVy() * time, Tf.getY() - Ts.getY()))
	        return TrajectoryInfo.INFINITY;
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(u, time);
	    if (!isGoal(Ts.move(trajectory))) {
	    	logger.severe("Cannot find a feasible solution one translation");
	    	throw new RuntimeException("Feasible: single translation Error");
	    }
	    return new TrajectoryInfo(trajectory);
	}
	
	/**
	 * Find a possible trajectory using one rotation
	 * @param u a rotation
	 * @return a trajectory
	 */
	private TrajectoryInfo singleRotation(Control u) {
		logger.finest("In single plan with one rotation " + u);
	    Homogeneous cs = Ts.transform(new Homogeneous(u));
	    Homogeneous cf = Tf.transform(new Homogeneous(u));
	    // If the rotation center are not the same, then
	    // we can not reach the goal by only using u
	    if (!cs.samePoint(cf))
	        return TrajectoryInfo.INFINITY;
	    double angleS = Ts.getTheta();
	    double angleF = Tf.getTheta();
	    double time = Utility.timeToAngle(angleS, angleF, u.getOmega());
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(u, time);
	    if (!isGoal(Ts.move(trajectory))) {
	    	logger.severe("Cannot find a feasible solution one rotation");
	    	throw new RuntimeException("Feasible: single rotation Error");
	    }
	    return new TrajectoryInfo(trajectory);
	}
	
	/**
	 * Find a feasible trajectory using only two controls
	 * @return a trajectory
	 */
	private TrajectoryInfo pairPlan() {
		logger.fine("In pair plan");
		return U.controlStream().reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, u1) -> 
                                        {  TrajectoryInfo sol = pairPlan(u1); 
                                           return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2); 
	}
	
	/**
	 * Find a feasible trajectory using only two controls with one of them is u1
	 * @param u1 a control
	 * @return a trajectory
	 */
	private TrajectoryInfo pairPlan(Control u1) {
		return U.controlStream().filter(u2 -> !u1.equals(u2))
				                .reduce(TrajectoryInfo.INFINITY, 
                                        (currentMin, u2) -> 
                                        {  TrajectoryInfo sol = pairPlan(u1, u2); 
                                           return sol.compareSolution(currentMin) < 0 ? sol : currentMin; },
                                        (solution1, solution2) -> solution1.compareSolution(solution2) < 0 ? solution1 : solution2); 
	}
	
	/**
	 * Find a feasible trajectory using two controls u1 and u2
	 * @param u1 a control
	 * @param u2 a control
	 * @return a trajectory
	 */
	private TrajectoryInfo pairPlan(Control u1, Control u2) {
		logger.finer("In pair plan " + u1 + " " + u2);
		if (u1.isTranslation() && u2.isTranslation())
			return twoTranslations(u1, u2);
	    else if (u1.isRotation() && u2.isRotation())
	    	return twoRotations(u1, u2);
	    else if (u1.isRotation())
	    	return rotationAndTranslation(u1, u2);
	    else
	        return rotationAndTranslation(u2, u1);
	}
	
	/**
	 * Find a possible trajectory using two translations
	 * @param us a translation
	 * @param uf a translation
	 * @return a trajectory
	 */
	private TrajectoryInfo twoTranslations(Control us, Control uf) {
		logger.finest("In pair plan with two controls" + us + " " + uf);
		if (!Utility.angleEqual(Ts.getTheta(), Tf.getTheta()))
			return TrajectoryInfo.INFINITY;

	    Control usW = us.toWorld(Ts);
	    Control ufW = uf.toWorld(Tf);

	    double det = usW.getVx() * ufW.getVy() - ufW.getVx() * usW.getVy();

	    if (Utility.isZero(det))
	        return TrajectoryInfo.INFINITY;

	    double dx = Tf.getX() - Ts.getX();
	    double dy = Tf.getY() - Ts.getY();

	    /**
	     * We want to solve a system of equation:
	     * t0 * v0x + tf * vfx = dx
	     * t0 * v0y + tf * vfy = dy                                           
	     */

	    double ad1 = dx * ufW.getVy() - ufW.getVx() * dy;
	    double ad2 = usW.getVx() * dy - dx * usW.getVy();

	    double ts = ad1 / det;
	    double tf = ad2 / det;
	    if (Utility.isZero(ts))
	        ts = 0.0;
	    if (Utility.isZero(tf))
	        tf = 0.0;
	    if (ts < 0.0 || tf < 0.0)
	    	return TrajectoryInfo.INFINITY;
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(us, ts);
	    trajectory.addControl(uf, tf);
	    if (!isGoal(Ts.move(trajectory))) {
	    	logger.severe("Cannot find a feasible solution with two translations");
	    	throw new RuntimeException("Feasible: two translations Error");
	    }
	    return new TrajectoryInfo(trajectory);
	}
	
	/**
	 * Find a feasible trajectory using a rotation u1 and a translation u2
	 * @param u1 a rotation
	 * @param u2 a translation
	 * @return a trajectory
	 */
	private TrajectoryInfo rotationAndTranslation(Control u1, Control u2) {
		logger.finest("In pair plan with one rotation and one translation " + u1 + " " + u2);
	    Control u2s = u2.toWorld(Ts);
	    Control u2f = u2.toWorld(Tf);
	    
	    double th2s = Math.atan2(u2s.getVy(), u2s.getVx());
	    double th2f = Math.atan2(u2f.getVy(), u2f.getVx());
	    Homogeneous c1 = new Homogeneous(u1);
	    Point2D c1s = Ts.transform(c1).toPoint();
	    Point2D c1f = Tf.transform(c1).toPoint();
	    Point2D vcsf = new Point2D.Double(c1f.getX() - c1s.getX(), c1f.getY() - c1s.getY());
	    double thsf = Math.atan2(vcsf.getY(), vcsf.getX());
	    double ta = Utility.timeToAngle(th2s, thsf, u1.getOmega());
	    double tb = Utility.timeToAngle(thsf, th2f, u1.getOmega());
	    double tc = Utility.distanceToOrigin(vcsf) / u2.getVelocity();
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(u1, ta);
	    trajectory.addControl(u2, tc);
	    trajectory.addControl(u1, tb);
	    if (!isGoal(Ts.move(trajectory))) {
	    	logger.severe("Cannot find a feasible solution with one rotation and one translation");
	    	throw new RuntimeException("Feasible: rotation and translation Error");
	    }
	    return new TrajectoryInfo(trajectory);
	}
	
	/**
	 * Find a possible trajectory using two rotations u1 and u2
	 * @param u1 a rotation
	 * @param u2 a rotation
	 * @return a trajectory
	 */
	private TrajectoryInfo twoRotations(Control u1, Control u2) {
		logger.finest("In pair plan with two rotations " + u1 + " " + u2);
	    Control u2v = new Control(u2.getVx(), u2.getVy(), 0.0);
	    Control u2s = u2v.toWorld(Ts);
	    Control u2f = u2v.toWorld(Tf);
	    
	    double th2s = Math.atan2(u2s.getVy(), u2s.getVx());
	    double th2f = Math.atan2(u2f.getVy(), u2f.getVx());

	    Point2D c1s = u1.rotationCenter(Ts);
	    Point2D c1f = u1.rotationCenter(Tf);
	    Point2D vcsf = new Point2D.Double(c1f.getX() - c1s.getX(), c1f.getY() - c1s.getY());
	    
	    double thsf = Math.atan2(vcsf.getY(), vcsf.getX());
	    double ta;
	    double tb;
	    if (Utility.isZero(u2v.getVelocity())) {
	    	ta = Utility.timeToAngle(Ts.getTheta(), thsf, u1.getOmega());
	    	tb = Utility.timeToAngle(thsf, Tf.getTheta(), u1.getOmega());
	    }
	    else {
	    	ta = Utility.timeToAngle(th2s, thsf, u1.getOmega());
	    	tb = Utility.timeToAngle(thsf, th2f, u1.getOmega());
	    }
	    Transformation Ta = Ts.move(u1, ta);
	    Transformation Tb = Tf.move(u1.reverse(), tb);
	    Trajectory middle = transformrot(u2, u1, Ta, Tb);
	    if (middle == null)
	    	return TrajectoryInfo.INFINITY;
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(u1, ta);
	    trajectory.append(middle);
	    trajectory.addControl(u1, tb);
	    
	    if (!isGoal(Ts.move(trajectory))) {
	    	System.out.println(Ts);
	    	System.out.println(trajectory);
	    	System.out.println(Ts.move(trajectory));
	    	double error = Tf.toPoint().distance(Ts.move(trajectory).toPoint());
	    	logger.warning("Numerical error in two rotations, error is " + error);
	    	logger.warning("Numerical error in two rotations, initial configuration is " + Ts);
	    	logger.warning("Numerical error in two rotations, trajectory is " + trajectory);
	    	logger.warning("Numerical error in two rotations, final configuration is " + Ts.move(trajectory));
	    	// For some cases, the computation is not exact..
	    	//if (error > Utility.EPSILON * 100)
	    	//	throw new RuntimeException("Feasible: two rotations Error");
	    }
	    return new TrajectoryInfo(trajectory);
	}
	
	/**
	 * Move from Ta to Tb using u1 and u2, where Ta and Tb only differ in x and y coordinate
	 * @param u1 a control
	 * @param u2 a control
	 * @param Ta a transformation
	 * @param Tb a transformation
	 * @return a trajectory
	 */
	private static Trajectory transformrot(Control u1, Control u2, Transformation Ta, Transformation Tb) {
		Homogeneous c1 = new Homogeneous(u1);
		Homogeneous c2 = new Homogeneous(u2);
		double d12 = c1.distance(c2);
		if (Utility.isZero(d12))
			return null;
		Point2D c1a = u1.rotationCenter(Ta);
		Point2D c1b = u1.rotationCenter(Tb);
		Point2D vcab = new Point2D.Double(c1b.getX() - c1a.getX(), c1b.getY() - c1a.getY());
		double dab = Utility.distanceToOrigin(vcab);
		double atan = Math.atan2(vcab.getY(), vcab.getX());
		Trajectory trajectory = new Trajectory();
	    if (2.0 * d12 < dab || Utility.absEqual(2.0 * d12, dab)) {
	    	double t = Utility.timeToLine(Ta, u1, u2, atan);
	    	trajectory.addControl(u1, t);
	    	trajectory.addControl(u2, Math.PI / Math.abs(u2.getOmega()));
	        dab -= 2.0 * d12;
	        while (2.0 * d12 < dab || Utility.absEqual(2.0 * d12, dab)) {
	        	trajectory.addControl(u1, Math.PI / Math.abs(u1.getOmega()));
		    	trajectory.addControl(u2, Math.PI / Math.abs(u2.getOmega()));
	            dab -= 2.0 * d12;
	        }
	        Ta = Ta.move(trajectory);
	        t = Utility.timeToAngle(Ta.getTheta(), Tb.getTheta(), u1.getOmega());
	        trajectory.addControl(u1, t);
	        Ta = Ta.move(u1, t);
	    }
	    if (Utility.isZero(dab))
	        return trajectory;
	    // At this point, we have two states Ta, Tb that satisfy the initial conditions of the function and 
	    // are also closer than d12
	    
	    double coseps = dab/(2.0 * d12);
	    if (Utility.isZero(coseps))
	    	coseps = 0.0;
	    else if (Utility.absEqual(coseps, 1.0))
	    	coseps = 1.0;
	    double theta = Math.acos(coseps);
	    Trajectory positive = threePointMove(Ta, Tb, u1, u2, atan + theta, atan - theta);
	    Trajectory negative = threePointMove(Ta, Tb, u1, u2, atan - theta, atan + theta);
	    trajectory.append(positive.totalTime() < negative.totalTime() ? positive : negative);
	    return trajectory;
	}
	
	/**
	 * Move from Ta to Tb
	 * @param Ta
	 * @param Tb
	 * @param u1
	 * @param u2
	 * @param dir1
	 * @param dir2
	 * @return
	 */
	private static Trajectory threePointMove(Transformation Ta, Transformation Tb, Control u1, Control u2, double dir1, double dir2) {
		Trajectory trajectory = new Trajectory();
		double ta = Utility.timeToLine(Ta, u1, u2, dir1);
	    trajectory.addControl(u1, ta);
	    double tb = Utility.timeToLine(Ta.move(trajectory), u2, u1, dir2);
	    trajectory.addControl(u2, tb);
	    double tf = Utility.timeToAngle(Ta.move(trajectory).getTheta(), Tb.getTheta(), u1.getOmega());
	    trajectory.addControl(u1, tf);
	    return trajectory;
	}
}
