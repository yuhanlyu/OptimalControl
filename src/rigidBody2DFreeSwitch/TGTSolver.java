package rigidBody2DFreeSwitch;

import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlSet;
import optimalControl.Homogeneous;
import optimalControl.Interval;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.TransformationTime;
import optimalControl.Utility;

/**
 * Solver for TGT trajectories
 * @author yu-hanlyu
 * There are some bugs.
 */
public class TGTSolver extends OptimalTrajectorySolver {
	
	public TGTSolver(ControlSet U, Transformation Ts) {
		super(U, Ts);
	}

	@Override
	public TrajectoryInfo solve() {
		solution = TGT();
		minTime = solution.getTime();
		return solution;
	}
	
	/**
	 * Finding the optimal TGT trajectory
	 * @return a trajectory
	 */
	private TrajectoryInfo TGT() {
		TrajectoryInfo currentMin = TrajectoryInfo.INFINITY;
		for (Control us : U) {
			if (us.isRotation())
				continue;
			for (Control uf : U) {
	        	if (uf.isRotation())
	        		continue;
	        	TrajectoryInfo sol = TGT(us, uf);
	        	if (sol.compareSolution(currentMin) < 0)
		            currentMin = sol;
	        }
		}
		return currentMin;
	}

	/**
	 * Find a TGT trajectory with first control us and the last control uf
	 * @param us first control
	 * @param uf final control
	 * @return a trajectory
	 */
	private TrajectoryInfo TGT(Control us, Control uf) {
	    ControlLine controlLine = TGTControlLine(us, uf);
	    if (!controlLine.isValid())
	        return TrajectoryInfo.INFINITY;
	    return TGT(us, uf, controlLine);
	}
	
	/**
	 * Find a TGT trajectory with first control us, the last control uf, and a control line
	 * @param us first control
	 * @param uf final control
	 * @param controlLine a control line
	 * @return a trajectory
	 */
	protected TrajectoryInfo TGT(Control us, Control uf, ControlLine controlLine) {
		Transformation TLW = new Transformation(controlLine);
		Transformation TsL = TLW.transform(Ts);
		Transformation afterUs = TsL;
		
	    if (U.isSustainable(TsL, us)) {
	        TransformationTime spair = U.completeSegment(TsL, us);
	        afterUs = spair.getTransformation();
	    }
	    
	    Transformation TfL = TLW.transform(Tf);
	    Control usL = us.toWorld(TsL);
	    Control ufL = uf.toWorld(TfL);

	    Transformation beforeUf = afterUs;
	    Trajectory period = U.getPeriod(beforeUf);
	    // Find the configuration before applying Uf
	    int index = 0;
	    for (index = 0; index < period.size(); ++index) {
	        Control u = period.getControl(index);
	        // Find the first translation
	        if (u.isTranslation())
	            break;
	        beforeUf = beforeUf.move(u, period.getDuration(index));
	    }
	    
	    // If cannot find a translation or the first translation is not uf
	    // then it is not a valid TGT
	    if (index == period.size()
	     || !period.getControl(index).equals(uf)
	     || !Utility.angleEqual(beforeUf.getTheta(), TfL.getTheta()))
	        return TrajectoryInfo.INFINITY;
	    
	    double[] t0AndTf = tsAndTf(TsL, afterUs, beforeUf, TfL, usL, ufL);
	    double ts = t0AndTf[0];
	    double tf = t0AndTf[1];
	    if (!Double.isFinite(ts))
	    	return TrajectoryInfo.INFINITY;
	    
	    Trajectory trajectory = new Trajectory();
	    trajectory.addControl(us, ts);
	    for (int i = 0; i < index; ++i) {
	    	trajectory.addControl(period.getControl(i), period.getDuration(i));
	    }
	    trajectory.addControl(uf, tf);
	    if (!isGoal(Ts.move(trajectory))) {
	    	throw new RuntimeException("TGT Error");
	    }
	    TrajectoryInfo result = TrajectoryInfo.createTGT(trajectory, controlLine);
	    foundOneSolution(result);
	    return result;
	}
	
	/**
	 * Determine the duration for the first control and the last control
	 * @param TsL The initial configuration
	 * @param afterUs the control after us
	 * @param beforeUf the control before uf
	 * @param TfL the final configuration
	 * @param usL the translation at the initial configuration in the control line frame
	 * @param ufL the translation at the final configuration in the control line frame
	 * @return time for the first control and the last control
	 */
	protected static double[] tsAndTf(Transformation TsL,
                                    Transformation afterUs,
                                    Transformation beforeUf,
                                    Transformation TfL,
                                    Control usL, Control ufL) {
		double dx = TfL.getX() - TsL.getX();
	    double dy = TfL.getY() - TsL.getY();
	    double midDx = beforeUf.getX() - afterUs.getX();
	    double midDy = beforeUf.getY() - afterUs.getY();

	    /**
	     * We want to solve a system of equation:
	     * t0 * v0x + mid_dx + tf * vfx = dx
	     * t0 * v0y + mid_dy + tf * vfy = dy                                           
	     * which is identical
	     * v0x * t0 + vfx * tf = dx - mid_dx
	     * v0y * t0 + vfy * tf = dy - mid_dy
	     */

	    dx -= midDx;
	    dy -= midDy;
	    double det = usL.getVx() * ufL.getVy() - ufL.getVx() * usL.getVy();
	    double ad1 = dx * ufL.getVy()       - ufL.getVx() * dy;
	    double ad2 = ufL.getVx() * dy       - dx * usL.getVy();

	    double ts = ad1 / det;
	    double tf = ad2 / det;
	    if (Utility.isZero(ts))
	        ts = 0.0;
	    if (Utility.isZero(tf))
	        tf = 0.0;
	    if (ts < 0.0 || tf < 0.0) {
	    	ts = tf = Double.POSITIVE_INFINITY;
	    }
	    return new double[] {ts, tf};
	}
	
	/**
	 * Create a control line for a TGT trajectory
	 * @param us first control
	 * @param uf last control
	 * @return a control line
	 */
	private ControlLine TGTControlLine(Control us, Control uf) {
		Control usW = us.toWorld(Ts);
		Control ufW = uf.toWorld(Tf);
	    if (usW.close(ufW))
	        return ControlLine.NULL_LINE;
	    double theta = Math.atan2(usW.getVx() - ufW.getVx(), ufW.getVy() - usW.getVy());
	    double H = usW.getVx() * Math.cos(theta) + usW.getVy() * Math.sin(theta);
	    if (Utility.isZero(H)) {
	    	return ControlLine.NULL_LINE;
	    }
	    
	    if (H < 0) {
	        theta += Math.PI;
	        H = -H;
	    }

	    double k1 = Math.cos(theta);
	    double k2 = Math.sin(theta);

	    Interval range = computeK3Range(Ts, us, k1, k2);
	    // Do we need to check for uf??
	    if (range.getEnd() == Double.POSITIVE_INFINITY) {
	        if (range.getBegin() == Double.POSITIVE_INFINITY)
	            return ControlLine.NULL_LINE;
	        return new ControlLine(k1, k2, range.getBegin(), H);
	    } else if (range.getBegin() > range.getEnd())
	        return ControlLine.NULL_LINE;
	    if (Utility.absEqual(range.getBegin(), range.getEnd()))
	    	return ControlLine.NULL_LINE;
	    return new ControlLine(k1, k2, range.getEnd(), H);
	}
	
	/**
	 * Compute the range of k3
	 * @param T a configuration
	 * @param u1 a control
	 * @param k1 kx
	 * @param k2 ky
	 * @return the range of ktheta
	 */
	private Interval computeK3Range(Transformation T, Control u1, double k1, double k2) {
		double ub = Double.POSITIVE_INFINITY;
	    double lb = Double.NEGATIVE_INFINITY;
	    for (int i = 0; i < U.size(); ++i) {
	    	Control u2 = U.getControl(i);
	        if (u2.isTranslation())
	            continue;
	        double k3 = switchPointOnLine(T, u1, u2, k1, k2);
	        if (u2.getOmega() > 0.0) {
	            if (k3 < ub)
	                ub = k3;
	        } else if (k3 > lb)
	            lb = k3;
	    }
	    return new Interval(lb, ub);
	}

	/**
	 * Compute the value of k3 when the switch point of u1 and u2 is on the control line
	 * @param T a configuration
	 * @param u1 first control
	 * @param u2 next control
	 * @param k1 kx
	 * @param k2 ky
	 * @return ktheta
	 */
	private static double switchPointOnLine(Transformation T, Control u1, Control u2, double k1, double k2) {
		Homogeneous SP = T.transform(u1.switchPoint(u2));
	    return (k2 * SP.getX() - k1 * SP.getY()) / SP.getOmega();
	}
	
	/**
	 * Invoked when found one trajectory
	 * @param trajectory
	 */
	protected void foundOneSolution(TrajectoryInfo trajectory) {
	}
}
