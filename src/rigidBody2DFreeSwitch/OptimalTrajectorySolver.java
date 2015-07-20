package rigidBody2DFreeSwitch;
import java.awt.geom.Point2D;
import java.util.List;

import optimalControl.Configuration;
import optimalControl.ControlSet;
import optimalControl.Transformation;
import optimalControl.Utility;

/**
 * An abstract class for solver 
 * @author yu-hanlyu
 * Should chage to thread local
 */
public abstract class OptimalTrajectorySolver {
	public static final Transformation Tf = new Transformation(new Configuration(0, 0, 0));
	protected ControlSet U;
	protected Transformation Ts;
	protected TrajectoryInfo solution;
	protected double minTime;
	
	/**
	 * Constructor
	 * @param U a control set
	 * @param Ts initial configuration
	 * @param Tf final configuration
	 */
	public OptimalTrajectorySolver(ControlSet U, Transformation Ts) {
		this.U = U;
		this.Ts = Ts;
	}
	
	/**
	 * Return the solution trajectory
	 * @return the solution trajectory
	 */
	public final TrajectoryInfo getSolution() { 
		return solution; 
	}
	
	/**
	 * Return the minimum time
	 * @return the minimum time
	 */
	public final double getMinTime() {
		return minTime;
	}
	
	/**
	 * Find an optimal trajectory
	 */
	public abstract TrajectoryInfo solve();
	
	/**
	 * Find all optimal trajectories that satisfies necessary conditions
	 * @return all trajectories
	 */
	public List<TrajectoryInfo> getAllTrajectories() {
		return null;
	}
	
	/**
	 * Test whether T is goal
	 * @param T a configuration
	 * @return true if T is approximately equal to the goal
	 */
	public static final boolean isGoal(Transformation T) {
		double distX = T.getX() - Tf.getX();
		double distY = T.getY() - Tf.getY();
		return Utility.isZero(Utility.distanceToOrigin(new Point2D.Double(distX, distY)))
		    && Utility.angleEqual(T.getTheta(), Tf.getTheta());
	}
}
