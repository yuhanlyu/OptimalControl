package optimalControl;
/**
 * @author yu-hanlyu
 *
 */

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
	private List<Control> controls = new ArrayList<>();
	private List<Double> durations = new ArrayList<>();
	private static final double PERTUBATION = 0.1;
	
	/**
	 * 
	 */
	public Trajectory() {}
	
	/**
	 * @param index the index of the controls
	 * @return the index-th control
	 */
	public Control getControl(int index) {
		return controls.get(index);
	}
	
	/**
	 * @param index the index of the controls
	 * @return the index-th control
	 */
	public double getDuration(int index) {
		return durations.get(index);
	}
	
	/**
	 * @return the number of segments in the trajectory
	 */
	public int size() {
		return controls.size();
	}
	
	/**
	 * @param u: a control
	 * @param duration: corresponding duration
	 */
	public void addControl(Control u, double duration) {
		controls.add(u);
		durations.add(duration);
		if (duration < 0) {
			throw new IllegalArgumentException("Duration cannot be negative " + duration);
		}
	}
	
	/**
	 * Append a trajectory to this trajectory
	 * @param trajectory a trajectory
	 */
	public void append(Trajectory trajectory) {
		for (int i = 0; i < trajectory.size(); ++i) {
			addControl(trajectory.getControl(i), trajectory.getDuration(i));
		}
	}
	
	/**
	 * Remove the last control and its duration
	 */
	public void removeControl() {
		controls.remove(controls.size() - 1);
		durations.remove(durations.size() - 1);
	}
	
	/**
	 * Compute the total duration
	 * sum function implements KaHan summation algorithm
	 * @return the total duration
	 */
	public double totalTime() {
		return durations.stream().mapToDouble(t -> t).sorted().sum();
	}
	
	/**
	 * Reflect trajectory
	 * @return reflected trajectory
	 */
	public Trajectory reflect() {
		Trajectory result = new Trajectory();
		for (int i = size() - 1; i >= 0; --i)
			result.addControl(getControl(i).reverse(), getDuration(i));
		return result;
	}
	
	/**
	 * Randomly permute the duration
	 */
	public void perturbTime() {
		int rindex = (int)(Math.random() * durations.size());
		double duration = durations.get(rindex);
		duration += (Math.random() - 0.5)* PERTUBATION;
		durations.set(rindex, duration);
	}
	
	/**
	 * Left rotate by one position
	 */
	public void leftRotate()  {
		Double duration = durations.get(0);
		Control control = controls.get(0);		
		durations.remove(0);
		controls.remove(0);
		addControl(control, duration);
	}
	
	/**
	 * String representation of the trajectory
	 */
	@Override
	public String toString() {
		StringBuilder result = new StringBuilder();
		for (int i = 0; i < controls.size(); ++i) {
			result.append(controls.get(i) + " " + durations.get(i) + '\n');
		}
		return result.toString();
	}
	
	/**
	 * Compute the cost of the trajectory
	 * @param switchCost the cost of switching controls
	 * @return
	 */
	public double getCost(double switchCost) {
		return totalTime() + switchCost * (size() - 1);
	}
}
