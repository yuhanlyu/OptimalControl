package rigidBody2DFreeSwitch;
import optimalControl.ControlLine;
import optimalControl.ControlStructure;
import optimalControl.Trajectory;
import optimalControl.Utility;

/**
 * Information of a feasible solution (distance to the goal is approximately zero)
 * @author yu-hanlyu
 *
 */
public class TrajectoryInfo {
	public static TrajectoryInfo INFINITY = null;
	static {
		Trajectory trajectory = new Trajectory();
		trajectory.addControl(null, Double.POSITIVE_INFINITY);
		INFINITY = new TrajectoryInfo(trajectory);
	}
	private static double switchCost = 0; // Used in comparison
	private Trajectory trajectory;
	private ControlStructure structure;
	private ControlLine controlLine;
	private boolean isTGT = false;
	private boolean isGeneric = false;
	private boolean isWhirl = false;
	private boolean isSingular = false;
	
	/**
	 * Constructor
	 * @param trajectory
	 */
	public TrajectoryInfo(Trajectory trajectory) {
		this.trajectory = trajectory;
	}
	
	public static void setSwitchCost(double SC) {
		switchCost = SC;
	}
	
	/**
	 * Create a generic trajectory info
	 * @param trajectory
	 * @param structure
	 * @return
	 */
	public static TrajectoryInfo createGeneric(Trajectory trajectory, ControlStructure structure, ControlLine controlLine) {
		TrajectoryInfo result = new TrajectoryInfo(trajectory);
		result.structure = structure;
		result.controlLine = controlLine;
		result.isGeneric = true;
		return result;
	}
	
	/**
	 * Create a TGT trajectory info
	 * @param trajectory
	 * @return
	 */
	public static TrajectoryInfo createTGT(Trajectory trajectory, ControlLine controlLine) {
		TrajectoryInfo result = new TrajectoryInfo(trajectory);
		result.controlLine = controlLine;
		result.isTGT = true;
		return result;
	}
	
	/**
	 * Create a singular trajectory info
	 * @param trajectory
	 * @return
	 */
	public static TrajectoryInfo createSingular(Trajectory trajectory, ControlLine controlLine) {
		TrajectoryInfo result = new TrajectoryInfo(trajectory);
		result.controlLine = controlLine;
		result.isSingular = true;
		return result;
	}
	
	/**
	 * Create a whirl trajectory info
	 * @param trajectory
	 * @return
	 */
	public static TrajectoryInfo createWhirl(Trajectory trajectory) {
		TrajectoryInfo result = new TrajectoryInfo(trajectory);
		result.isWhirl = true;
		return result;
	}
	
	/**
	 * Return the trajectory
	 * @return
	 */
	public Trajectory getTrajectory() {
		return trajectory;
	}
	
	/**
	 * Return the structure, if it is generic
	 * @return
	 */
	public ControlStructure getStructure() {
		return structure;
	}
	
	/**
	 * Test whether the trajectory is generic
	 * @return
	 */
	public boolean isGeneric() {
		return isGeneric;
	}
	
	/**
	 * Test whether the trajectory is TGT
	 * @return
	 */
	public boolean isTGT() {
		return isTGT;
	}
	
	/**
	 * Test whether the trajectory is singular
	 * @return
	 */
	public boolean isSingular() {
		return isSingular;
	}
	
	/**
	 * Test whether the trajectory is whirl
	 * @return
	 */
	public boolean isWhirl() {
		return isWhirl;
	}
	
	/**
	 * Test whether the trajectory is not generic, TGT, singular, and whirl
	 * @return
	 */
	public boolean isFeasible() {
		return !isGeneric() && !isTGT() && !isSingular() && !isWhirl();
	}
	
	/**
	 * Return the time for the trajectory
	 * @return
	 */
	public double getTime() {
		return trajectory.totalTime();
	}
	
	/**
	 * Return the cost for the trajectory
	 * @param switchCost the cost of switching controls
	 * @return
	 */
	public double getCost(double switchCost) {
		return trajectory.getCost(switchCost);
	}
	
	/**
	 * Compare two solutions
	 * @param lhs
	 * @param rhs
	 * @return negative if this is better then the parameter, else positive
	 */
	public int compareSolution(TrajectoryInfo solution) {
		return getCost(switchCost) <= solution.getCost(switchCost) ? -1 : 1;
	}
	
	/**
	 * Test whether two solutions have almost the same time
	 * @param solution
	 * @return
	 */
	public boolean close(TrajectoryInfo solution) {
		return Utility.absEqual(getTime(), solution.getTime());
	}
	
	/**
	 * Return the controlLine
	 * @return
	 */
	public ControlLine getControlLine() {
		return hasControlLine() ? controlLine : null;
	}
	
	/**
	 * 
	 * @return
	 */
	private boolean hasControlLine() {
		return isGeneric || isSingular || isTGT;
	}
	/**
	 * Test whether the trajectory is valid or not
	 * @return
	 */
	public boolean isValid() {
		return Double.isFinite(getTime());
	}
}
