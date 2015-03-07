package robotModel;
import java.awt.Shape;

import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.ControlStructure;
import optimalControl.Trajectory;
import rigidBody2DFreeSwitch.Synthesis;
import rigidBody2DFreeSwitch.TrajectoryInfo;

public class Robot {
	private ControlSet U;
	private Shape shape;  //Rectangle2D.Double(-0.6, -0.3, 1.2, 0.6);
	
	/**
	 * @param index specify the i-th control
	 * @return the i-th control
	 */
	public Control getControl(int index) {
		return U.getControl(index);
	}
	
	/**
	 * @return the shape representing the robot
	 */
	public Shape getShape() {
		return shape;
	}
	
	/**
	 * 
	 * @return the control set
	 */
	public ControlSet getControlSet() {
		return U;
	}
	
	/**
	 * Setup the shape and control set
	 * @param shape 
	 * @param U
	 */
	public Robot(ControlSet U, Shape shape) {
		this.shape = shape;
		this.U = U;
	}
	
	/**
	 * Analyze the trajectory type
	 * @param solution the trajectory to be analyzed
	 * @return an integer representing the type
	 */
	public final int analyzeTrajectory(TrajectoryInfo solution) {
		if (solution == null)
			return Synthesis.NO_SOLUTION;
		Trajectory trajectory = solution.getTrajectory();
		if (solution.isGeneric()) {
			return analyzeGeneric(solution.getStructure());
		} else if (solution.isTGT()) {
			return analyzeTGT(trajectory);
		} else if (solution.isWhirl()) {
			return analyzeWhirl(trajectory);
		} else if (solution.isSingular()) {
			return analyzeSingular(trajectory);
		} else {
			return analyzeFeasible(trajectory);
		}
	}
	
	/**
	 * Analyze a generic trajectory
	 * @param structure
	 * @return an integer representing the type
	 */
	protected int analyzeGeneric(ControlStructure structure) {
		return Synthesis.NO_SOLUTION;
	}
	
	/**
	 * Analyze a TGT trajectory
	 * @param trajectory
	 * @return an integer representing the type
	 */
	protected static int analyzeTGT(Trajectory trajectory) {
		return Synthesis.TGT_SOLUTION;
	}
	
	/**
	 * Analyze a whirl trajectory
	 * @param trajectory
	 * @return an integer representing the type
	 */
	protected static int analyzeWhirl(Trajectory trajectory) {
		return Synthesis.WHIRL_SOLUTION;
	}
	
	/**
	 * Analyze a singular trajectory
	 * @param trajectory
	 * @return an integer representing the type
	 */
	protected static int analyzeSingular(Trajectory trajectory) {
		return Synthesis.SINGULAR_SOLUTION;
	}
	
	/**
	 * Analyze other trajectories
	 * @param trajectory
	 * @return an integer representing the type
	 */ 
	protected static int analyzeFeasible(Trajectory trajectory) {
		return Synthesis.FEASIBLE_SOLUTION;
	}
}
