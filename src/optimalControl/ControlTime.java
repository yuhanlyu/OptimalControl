package optimalControl;

/**
 * A class storing a control with time
 * @author yu-hanlyu
 *
 */
public class ControlTime {
	private Control control;
	private double time;
	
	/**
	 * Constructor
	 * @param control a control
	 * @param time duration
	 */
	public ControlTime(Control control, double time) {
		this.control = control;
		this.time = time;
	}
	
	/**
	 * Return the control
	 * @return Control
	 */
	public Control getControl() {
		return control;
	}
	
	/**
	 * Return the time
	 * @return time
	 */
	public double getTime() {
		return time;
	}
}
