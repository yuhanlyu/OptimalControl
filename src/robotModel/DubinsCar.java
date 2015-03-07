package robotModel;
import java.awt.Shape;
import java.awt.geom.Rectangle2D;
import java.util.Arrays;
import java.util.List;

import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.ControlStructure;
import rigidBody2DFreeSwitch.Synthesis;

/**
 * A class representing Dubin's car
 * @author yu-hanlyu
 *
 */
public class DubinsCar extends Robot {
	private static final Shape shape = new Rectangle2D.Double(-0.6, -0.3, 1.2, 0.6);
	private static final Control[] controls = {new Control(1, 0, 0), 
		                                       new Control(1, 0, 1), 
		                                       new Control(1, 0, -1)};
	private static final List<Double> critHs = Arrays.asList(new Double(1.0));
	
	/**
	 * Constructor
	 */
	public DubinsCar() {
		super(new ControlSet(Arrays.asList(controls), critHs, 1), shape);
	}
	
	@Override
	protected int analyzeGeneric(ControlStructure structure) {
		if (structure.size() == 2 && structure.containSwitch(controls[1], controls[2])) {
	        if (structure.getControl(0).equals(controls[1]))
	            return 1;
	        else if (structure.getControl(0).equals(controls[2]))
	            return 2;
	    }
	    return Synthesis.NO_SOLUTION; // Cannot happen
	}
}
