package optimalControl;

/**
 * A class storing a control with time
 * @author yu-hanlyu
 *
 */
public class ControlTime {
	private Control control;
	private double time;
	
	public ControlTime(Control control, double time) {
		this.control = control;
		this.time = time;
	}
	
	public Control getControl() {
		return control;
	}
	
	public double getTime() {
		return time;
	}
}
