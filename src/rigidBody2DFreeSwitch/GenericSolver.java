package rigidBody2DFreeSwitch;
import java.util.ArrayList;
import java.util.List;

import optimalControl.Configuration;
import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlLineFactory;
import optimalControl.ControlSet;
import optimalControl.ControlStructure;
import optimalControl.DistanceTime;
import optimalControl.DistanceTimeL;
import optimalControl.Interval;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.TransformationTime;
import optimalControl.Utility;
import robotModel.OmniDrive;

/**
 * An abstract class for generic trajectory solver
 * @author yu-hanlyu
 *
 */
public class GenericSolver extends OptimalTrajectorySolver {
	protected DistanceMinimizer minimizer;
	protected GenericInfo optimalSolutionInfo;
	protected boolean animate;
	
	/**
	 * Constructor
	 * @param U a control set
	 * @param Ts initial configuration
	 * @param delta gap along boundaries
	 * @param distError maximum distance error
	 * @param timeError maximum time error
	 */
	public GenericSolver(ControlSet U, Transformation Ts,
			                           DistanceMinimizer minimizer) {
		super(U, Ts);
		this.minimizer = minimizer;
	}
	
	/**
	 * 
	 */
	@Override
	public TrajectoryInfo solve() {
		animate = false;
		solution = TrajectoryInfo.INFINITY;
		optimalSolutionInfo = FreeGenericInfo.INFINITY;
		generateTasks().stream()
		               .filter(task -> task != null)
		               .sorted()
		               .forEach(task -> processTask(task));
		if (optimalSolutionInfo != FreeGenericInfo.INFINITY) {
			solution = TrajectoryInfo.createGeneric(optimalSolutionInfo.getTrajectory(), 
			     	                                optimalSolutionInfo.getStructure(),
			     	                                optimalSolutionInfo.getControlLine());
			if (!isGoal(Ts.move(solution.getTrajectory()))) {
				solution = TrajectoryInfo.INFINITY;
				optimalSolutionInfo = FreeGenericInfo.INFINITY;
			}
			minTime = solution.getTrajectory().totalTime();
		}
		return solution;
	}
	
	/**
	 * Produce animation
	 * @return a trajectory
	 */
	public TrajectoryInfo solveWithAnimation() {
		animate = true;
		return solve();
	}
	
	/**
	 * Process one task with animation
	 * Here, I just hard code the robot
	 * Moreover, if there are multiple tasks, then only the result of the last one will be stored
	 * @param task minimization task
	 */
	protected void processTask(DistanceMinimization task) {
		if (animate == true) {
			List<Trajectory> trajList = new ArrayList<>();
			List<ControlLine> controlLineList = new ArrayList<>();
			minimizer.minimize(task, trajList, controlLineList);
			minimizer.saveFile(new Configuration(Ts), new OmniDrive(), trajList, controlLineList);
		}
		GenericInfo solutionInfo = minimizer.minimize(task);
		if (minimizer.compareSolution(solutionInfo, optimalSolutionInfo) < 0) {
			optimalSolutionInfo = solutionInfo;
		}
	}
	
	/**
	 * Generate all tasks
	 * @return a list of tasks
	 */
	protected List<DistanceMinimization> generateTasks() {
		ArrayList<DistanceMinimization> result = new ArrayList<>();
		for (Control us : U) {
			for (Control uf : U) {
				if (us.isRotation() || uf.isRotation())
					result.addAll(generateTasks(us, uf));
			}
		}
		return result;
	}
	
	/**
	 * Generate all tasks with initial control us and final control uf
	 * @param us first control
	 * @param uf final control
	 * @return all tasks with initial control us and final control uf
	 */
	protected List<DistanceMinimization> generateTasks(Control us, Control uf) {
		ArrayList<DistanceMinimization> result = new ArrayList<>();
		ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, true);
		if (!factory.getRange().isEmpty())
			result.addAll(generateTasks(us, uf, factory));
		factory = new ControlLineFactory(U, Ts, us, Tf, uf, false);
		if (!factory.getRange().isEmpty())
			result.addAll(generateTasks(us, uf, factory));
		return result;
	}
	
	/**
	 * Generate all tasks with initial control us, final control uf, and the sign of control line
	 * @param us first control
	 * @param uf final control
	 * @param isPositive the sign of the control line
	 * @param factory the factory of the control line
	 * @return all tasks with initial control us and final control uf with the sign of control line
	 */
	protected List<DistanceMinimization> generateTasks(Control us, Control uf, ControlLineFactory factory) {
		return generateTasks(us, uf, factory, factory.getRange());
	}
	
	/**
	 * Generate all tasks with initial control us, final control uf, and the sign of control line
	 * @param us first control
	 * @param uf final control
	 * @param isPositive the sign of the control line
	 * @param factory the factory of the control line
	 * @param range of the Hamiltonian values
	 * @return all tasks with initial control us and final control uf with the sign of control line in the range
	 */
	protected List<DistanceMinimization> generateTasks(Control us, Control uf, ControlLineFactory factory, Interval range) {
	    int begin, end;
	    // Find the first critical value larger than lb
	    for (begin = 0; begin < U.getCriticalSize(); ++begin) {
	        if (U.getCriticalValue(begin) > range.getBegin())
	            break;
	    }
	    // Find the last critical value smaller than ub
	    for (end = U.getCriticalSize() - 1; end >= 0; --end) {
	        if (U.getCriticalValue(end) < range.getEnd())
	            break;
	    }
	    
	    ArrayList<DistanceMinimization> result = new ArrayList<>();
	    // Generate all ranges of H
	    if (begin == U.getCriticalSize() || end == -1 || begin == end + 1) {
	        result.add(generateTask(us, uf, factory, range.getBegin(), range.getEnd()));
	        return result;
	    }
	    
	    result.add(generateTask(us, uf, factory, range.getBegin(), U.getCriticalValue(begin)));
	    for (int i = begin; i < end; ++i)
	        result.add(generateTask(us, uf, factory, U.getCriticalValue(i), U.getCriticalValue(i+1)));
	    result.add(generateTask(us, uf, factory, U.getCriticalValue(end), range.getEnd()));
	    return result;
	}
	
	/**
	 * Generate a task with initial control us, final control uf, the sign of control line, and range of H 
	 * @param us first control
	 * @param uf last control
	 * @param isPositive the sign of the control line
	 * @param factory  factory of the control line
	 * @param lb lower bound of the range of H
	 * @param ub upper bound of the range of H
	 * @return a task with initial control us, final control uf, the sign of control line, and range of H
	 */
	protected DistanceMinimization generateTask(Control us, Control uf, ControlLineFactory factory, double lb, double ub) {
		if (Utility.absEqual(lb, ub))
			return null;
		double H = (lb + ub) * 0.5;
		DistanceFunctor metric = new FreeSwitchDistanceFunctor(factory, H);
		if (!metric.hasStructure())
			return null;
		DistanceTime distanceTime = metric.computeTrajectory(H).getDistanceTime();
		return new DistanceMinimization(metric, new Interval(lb, ub), distanceTime);
	}
	
	/**
	 * Compute the distance for a fixed control structure with different H values
	 * @author yu-hanlyu
	 *
	 */
	private class FreeSwitchDistanceFunctor implements DistanceFunctor {
		private Control afterUs;
		private Control afterUf;
		private ControlLineFactory factory;
		private ControlStructure structure;
		private long cycles;
		private int phaseLength;
		private ControlLineFactory.LFunctor sfunctor;
		private ControlLineFactory.LFunctor ffunctor;
		
		/**
		 * Constructor, the structure is determined by the Hamiltonian value
		 * If the Hamiltonian value is crossing the critical value, then the structure will change and
		 * the result is incorrect
		 * @param us first control
		 * @param uf last control
		 * @param factory control line factory
		 * @param H Hamiltonian value
		 */
		public FreeSwitchDistanceFunctor(ControlLineFactory factory, double H) {
			this.factory = factory;
			Transformation TLW = new Transformation(factory.getControlLine(H));
			Transformation TsL = TLW.transform(Ts);
			Transformation TfL = TLW.transform(Tf);
			TransformationTime spair = U.completeSegment(TsL, getUs());
			TransformationTime fpair = U.completeSegment(TfL, getUf());
			afterUf = U.sustainableControl(fpair.getTransformation());
						
			// Find the control structure
			structure = new ControlStructure(U.getPeriod(spair.getTransformation(), getUs()));
			if (structure.size() <= 1 || !structure.containSwitch(getUf(), afterUf)) {
				structure = null;
				return;
			}
			afterUs = structure.getControl(0);
			if (getUf().equals(afterUf)) {
				throw new RuntimeException("afterUs is not as expected");
			}
			phaseLength = computePhase();
			sfunctor = factory.getLFunctor(Ts, getUs(), afterUs);
			ffunctor = factory.getLFunctor(Tf, getUf(), afterUf);
		}
		
		/**
		 * Return the Lipschitz constant for the metric function
		 * @param H the Hamiltonian value
		 * @param multiplier a parameter to resolve numerical issues
		 * @return distance and time
		 */
		@Override
		public DistanceTimeL getL(double H, long multiplier) {
			DistanceTimeL sL = sfunctor.getL(H);
			DistanceTimeL fL = ffunctor.getL(H);
			DistanceTimeL structL = structure.getL(H);
			double timeL = sL.getTimeL() + fL.getTimeL() + multiplier * structL.getTimeL();
			double distL = sL.getDistanceL() + fL.getDistanceL() + multiplier * structL.getDistanceL();
			if (timeL < 0 || distL < 0 || Double.isInfinite(timeL) || Double.isInfinite(distL)) {
				throw new RuntimeException("Lipschitz Constant Error");
			}
			return new DistanceTimeL(distL, timeL);
		}
		
		/**
		 * Return the number of cycles
		 * @return
		 */
		@Override
		public long getCycles() {
			return cycles;
		}
		
		/**
		 * Return the first control
		 * @return the first control
		 */
		private Control getUs() {
			return factory.getUs();
		}
		
		/**
		 * Return the last control
		 * @return the last control
		 */
		private Control getUf() {
			return factory.getUf();
		}
		
		/**
		 * Determine whether structure exists
		 * @return true if structure exists
		 */
		@Override
		public boolean hasStructure() {
			return structure != null;
		}
		
		/**
		 * Compute the trajectory from Ts to Tf by using structure with the Hamiltonian H
		 * Assuming structure exists
		 * @param H the Hamiltonian value
		 * @return a trajectory with additional information
		 */
		@Override
		public GenericInfo computeTrajectory(double H) {
			ControlLine controlLine = factory.getControlLine(H);
			Transformation TLW = new Transformation(controlLine);
			Transformation TsL = TLW.transform(Ts);
			Transformation TfL = TLW.transform(Tf);
			
			// Complete the first segment and the last segment and compute distance
			TransformationTime spair = ControlSet.completeSegment(TsL, getUs(), afterUs);
			TransformationTime fpair = ControlSet.completeSegment(TfL, getUf(), afterUf);
			DistanceTime dist = getDistance(spair.getTransformation(), fpair.getTransformation(), H);
			
			// Adjust the distance and time considering the first control and the last control
			DistanceTime distanceTime = new DistanceTime(dist.getDistance(), dist.getTime() + spair.getTime() - fpair.getTime());
			// Information for the solution
			double ts = spair.getTime();
			Transformation TLR = spair.getTransformation();
			double tf = structure.buildTrajectory(H).getDuration(phaseLength - 1) - fpair.getTime();
			if (tf < 0 && Utility.isZero(tf))
				tf = 0;
			if (tf < 0) {
				throw new RuntimeException("Error: tf is smaller than 0");
			}
			if (cycles < 0) {
				throw new RuntimeException("Error: cycles is smaller than 0");
			}
			return new FreeGenericInfo.Builder(distanceTime, H)
			                                        .usAndUf(getUs(), getUf())
			                                        .tsAndTf(ts, tf)
			                                        .cyclesAndPhase(cycles, phaseLength)
			                                        .structure(structure)
			                                        .TLR(TLR)
			                                        .controlLine(controlLine)
			                                        .build();
		}
		
		/**
		 * Compute the distance from TsL to TfL by using structure
		 * @param TsL a configuration that is a starting of a control for afterUs
		 * @param TfL a configuration that is a starting of a control for afterUf
		 * @param H the Hamiltonian value
		 * @return distance and time
		 */
		private DistanceTime getDistance(Transformation TsL, Transformation TfL, double H) {
			DistanceTime period = structure.getDistanceTime(H);
			DistanceTime phase = structure.getPhaseDistnceTime(H, phaseLength);
			double d = TfL.getX() - TsL.getX() - phase.getDistance();
			double distance = computeCycles(d, period.getDistance());
			return new DistanceTime(distance, period.getTime() * cycles + phase.getTime());
		}
		
		/**
		 * Compute the index of the switch from uf to afterUf in the structure
		 * @return the index
		 */
		private int computePhase() {
			for (int i = 1; i <= structure.size(); ++i) {
		        if (getUf().equals(structure.getControl(i - 1)) 
		         && afterUf.equals(structure.getControl(i % structure.size()))) {
		            return i;
		        }
		    }
			return -1;
		}
		
		/**
		 * Compute minimum {|d - period_length * cycles| : cycles >= 0}
		 * @param dis distance to the goal
		 * @param periodLength length of the period
		 * @return minimum distance
		 */
		private double computeCycles(double d, double periodLength) {
			if (d * periodLength < 0) {
		        cycles = 0;
		        return Math.abs(d);
		    }

		    // If d and periodLength have the same sign
		    // cycles is either floor(d/period_length) or floor(d/period_length) + 1
		    cycles = (long)Math.floor(d / periodLength);
		    double dist1 = Math.abs(d - periodLength * cycles);
		    double dist2 = Math.abs(dist1 - periodLength);
		    if (dist1 < dist2)
		        return dist1;
			++cycles;
			return dist2;
		}

		@Override
		public List<GenericInfo> computeAllTrajectories(double H, double distError) {
			GenericInfo info = computeTrajectory(H);
			List<GenericInfo> result = new ArrayList<>();
			result.add(info);
			return result;
		}

		@Override
		public boolean hasMultipleSolution() {
			return false;
		}
	}
}
