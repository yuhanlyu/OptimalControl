package optimalControl;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A sequence of controls that represents a trajectory
 * @author yu-hanlyu
 *
 */
public class ControlStructure {
	private List<Control> controls = new ArrayList<>();
	private List<DTFunctor> dtFunctors = new ArrayList<>(); // Functors for computing distance-time and Lipschitz constants based on H
	
	/**
	 * Constructor
	 * @param trajectory
	 */
	public ControlStructure(Trajectory trajectory) {
		for (int i = 0; i < trajectory.size(); ++i) {
			controls.add(trajectory.getControl(i));
			Control pre = trajectory.getControl((i-1 + trajectory.size()) % trajectory.size());
			Control current = trajectory.getControl(i);
			Control next = trajectory.getControl((i+1) % trajectory.size());
			dtFunctors.add(DTFunctor.create(pre, current, next));
		}
	}
	
	/**
	 * Return the number of controls
	 * @return the number of controls
	 */
	public int size() {
		return controls.size();
	}
	
	/**
	 * Return the index-th control
	 * @param index the index of the control
	 * @return the index-th control
	 */
	public Control getControl(int index) {
		return controls.get(index);
	}
	
	/**
	 * Construct the trajectory corresponding the Hamiltonian value
	 * @param H the Hamiltonian value
	 * @return
	 */
	public Trajectory buildTrajectory(double H) {
		Trajectory result = new Trajectory();
		for (int i = 0; i < controls.size(); ++i) {
			double time = getDistanceTime(H, i).getTime();
			result.addControl(getControl(i), time);
		}
		return result;
	}
	
	/**
	 * Compute the distance and time from a configuration up the the index-th control
	 * @param H
	 * @param index
	 * @return
	 */
	public DistanceTime getDistanceTime(double H, int index) {
		return dtFunctors.get(index).getDistanceTime(H);
	}
	
	/**
	 * Compute the distance and time from a configuration for the whole structure
	 * @param H the Hamiltonian value
	 * @return
	 */
	public DistanceTime getDistanceTime(double H) {
		List<DistanceTime> dts = dtFunctors.stream().map(functor -> functor.getDistanceTime(H)).collect(Collectors.toList());
		return new DistanceTime(dts.stream().mapToDouble(dt -> dt.getDistance()).sum(),
                                dts.stream().mapToDouble(dt -> dt.getTime()).sum());
	}
	
	/**
	 * Compute the distance and time from a configuration no more than the phaseLength controls
	 * @param H the Hamiltonian value
	 * @param phaseLength
	 * @return
	 */
	public DistanceTime getPhaseDistnceTime(double H, int phaseLength) {
		List<DistanceTime> dts = new ArrayList<>();
		for (int i = 0; i < phaseLength; ++i) {
			dts.add(getDistanceTime(H, i));
		}
		return new DistanceTime(dts.stream().mapToDouble(dt -> dt.getDistance()).sum(),
				                dts.stream().mapToDouble(dt -> dt.getTime()).sum());
	}
	
	/**
	 * Compute the Lipschitz for the index-th control with a given H value
	 * @param H
	 * @param index
	 * @return
	 */
	public DistanceTimeL getL(double H, int index) {
		return dtFunctors.get(index).getL(H);
	}
	
	/**
	 * Compute the Lipschitz constant for the whole structure
	 * @param H
	 * @return
	 */
	public DistanceTimeL getL(double H) {
		List<DistanceTimeL> dtLs = dtFunctors.stream().map(functor -> functor.getL(H)).collect(Collectors.toList());
		return new DistanceTimeL(dtLs.stream().mapToDouble(dt -> dt.getDistanceL()).sum(),
                                 dtLs.stream().mapToDouble(dt -> dt.getTimeL()).sum());
	}
	
	/**
	 * Test whether there exists a switch from u1 to u2
	 * @param u1 one control
	 * @param u2 next control
	 * @return true if there is a switch from u1 to u2
	 */
	public boolean containSwitch(Control u1, Control u2) {
		for (int i = 0; i < size(); ++i)
	        if (u1.equals(getControl(i)) &&
	            u2.equals(getControl( (i+1) % size())))
	            return true;
	    return false;
	}
	
	/**
	 * String representation of a control structure
	 */
	@Override
	public String toString() {
		StringBuilder result = new StringBuilder();
		for (int i = 0; i < controls.size(); ++i) {
			result.append(controls.get(i) + "\n");
		}
		return result.toString();
	}
	
	/**
	 * For a configuration, compute the duration of the index-th control
	 * @param index the index of the control
	 * @param TLR a configuration in the control line frame
	 * @return the duration of the index-th control
	 */
	/*
	private double getTime(int index, Transformation TLR) {
		return ControlSet.timeToSwitch(TLR, 
				                       getControl(index), 
				                       getControl((index + 1) % size()), 
				                       false); 
	}*/
	
	/**
	 * Compute the distance and time from a configuration
	 * @param TLR a configuration in the control line frame
	 * @return the length of the structure in the x-coordinate and the duration
	 */
	/*
	public DistanceTime getDistanceTime(Transformation TLR) {
		double totalTime = 0;
		Transformation T = TLR;

	    for (int i = 0; i < controls.size(); ++i) {
	    	double time = getTime(i, T);
	        totalTime += time;
	        T = T.move(getControl(i), time);
	    }
	    return new DistanceTime(T.getX() - TLR.getX(), totalTime);
	}*/
	
	/**
	 * Compute the distance and time from a configuration no more than the the index-th control
	 * @param TLR a configuration in the control line frame
	 * @param index the index of the control
	 * @return the length of the structure in the x-coordinate and the duration
	 */
	/*
	public DistanceTime getDistanceTime(Transformation TLR, int index) {
		double totalTime = 0;
		Transformation T = TLR;

	    for (int i = 0; i < index; ++i) {
	    	double time = getTime(i, T);
	        totalTime += time;
	        T = T.move(getControl(i), time);
	    }
	    return new DistanceTime(T.getX() - TLR.getX(), totalTime);

	}*/
	
	/**
	 * Construct the trajectory corresponding the structure start at from a configuration
	 * @param TLR a configuration in the control line frame
	 * @return the trajectory corresponding the structure start at from a configuration
	 */
	/*
	public Trajectory buildTrajectory(Transformation TLR) {
		Trajectory result = new Trajectory();
		Transformation T = TLR;
		
		for (int i = 0; i < controls.size(); ++i) {
	    	double time = getTime(i, T);
	        T = T.move(getControl(i), time);
	        result.addControl(getControl(i), time);
	    }
		return result;
	}*/
}
