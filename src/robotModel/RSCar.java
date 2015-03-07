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
 * A class representing Reed-Shepp car
 * @author yu-hanlyu
 *
 */
public class RSCar extends Robot{
	private static final Shape shape = new Rectangle2D.Double(-0.6, -0.3, 1.2, 0.6);
	private static final Control[] controls = new Control[]{new Control(1, 0, 0), 
		                                                    new Control(1, 0, 1), 
		                                                    new Control(1, 0, -1),
		                                                    new Control(-1, 0, 0),
		                                                    new Control(-1, 0, 1),
		                                                    new Control(-1, 0, -1)};
	private static final List<Double> critHs = Arrays.asList(new Double(1.0));
	
	/**
	 * Constructor
	 */
	public RSCar() {
		super(new ControlSet(Arrays.asList(controls), critHs, 1), shape);
	}
	
	@Override
	protected int analyzeGeneric(ControlStructure structure) {
		if (structure.size() == 4 && structure.containSwitch(controls[1],  controls[2])) {
	        if (structure.getControl(0).equals(controls[1]) && structure.getControl(1).equals(controls[2]))
	            return 1;
	        else if (structure.getControl(0).equals(controls[2]) && structure.getControl(1).equals(controls[5]))
	            return 2;
	        else if (structure.getControl(0).equals(controls[5]) && structure.getControl(1).equals(controls[4]))
	            return 3;
	        else if (structure.getControl(0).equals(controls[4]) && structure.getControl(1).equals(controls[1]))
	            return 4;
	    } else if (structure.size() == 4 && structure.containSwitch(controls[1], controls[4])) {
	        if (structure.getControl(0).equals(controls[1]) && structure.getControl(1) == controls[4])
	            return 5;
	        else if (structure.getControl(0).equals(controls[4]) && structure.getControl(1).equals(controls[5]))
	            return 6;
	        else if (structure.getControl(0).equals(controls[5]) && structure.getControl(1).equals(controls[2]))
	            return 7;
	        else if (structure.getControl(0).equals(controls[2]) && structure.getControl(1).equals(controls[1]))
	            return 8;
	    }
	    return Synthesis.NO_SOLUTION; // Cannot happen
	}
}
