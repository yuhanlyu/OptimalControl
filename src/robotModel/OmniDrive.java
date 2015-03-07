package robotModel;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.util.Arrays;
import java.util.List;

import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.ControlStructure;
import rigidBody2DFreeSwitch.Synthesis;

/**
 * A class representing omnidirectional vehicle
 * @author yu-hanlyu
 *
 */
public class OmniDrive extends Robot {
	private static final double c = Math.sqrt(3.0) / 3.0;
	private static final double c2 = Math.sqrt(3.0) / 1.5;
	private static final Shape shape = new Ellipse2D.Double(-0.5, -0.5, 1, 1);
	private static final Control[] controls = {new Control(c, 1.0, 0.0),
        									   new Control(-c, -1.0, 0.0),
        									   new Control(-c, 1.0, 0.0),
        									   new Control(c, -1.0, 0.0),
        									   new Control(-c2, 0.0, 0.0),
        									   new Control(c2, 0.0, 0.0),
        									   new Control(0.0, 0.0, 1.0),
        									   new Control(0.0, 0.0, -1.0),
        									   new Control(0.0, -4.0/3.0, 1.0/3.0),
        									   new Control(0.0, 4.0/3.0, -1.0/3.0),
        									   new Control(c2, 2.0/3.0, 1.0/3.0),
        									   new Control(-c2, -2.0/3.0, -1.0/3.0),
        									   new Control(-c2, 2.0/3.0, 1.0/3.0),
        									   new Control(c2, -2.0/3.0, -1.0/3.0)};
	private static final List<Double> critHs = Arrays.asList(new Double(c2));
	
	/**
	 * Constructor
	 */
	public OmniDrive() {
		super(new ControlSet(Arrays.asList(controls), critHs, 2.0), shape);
	}
	
	@Override
	protected int analyzeGeneric(ControlStructure structure) {
		// Row trajectory
	    if (structure.size() == 6&& structure.containSwitch(controls[8], controls[6])) {
	        if (structure.getControl(0).equals(controls[8]) && structure.getControl(1).equals(controls[6]))
	            return 1;
	        else if (structure.getControl(0).equals(controls[6]) && structure.getControl(1).equals(controls[12]))
	            return 2;
	        else if (structure.getControl(0).equals(controls[12]) && structure.getControl(1).equals(controls[6]))
	            return 3;
	        else if (structure.getControl(0).equals(controls[6]) && structure.getControl(1).equals(controls[10]))
	            return 4;
	        else if (structure.getControl(0).equals(controls[10]) && structure.getControl(1).equals(controls[6]))
	            return 5;
	        else if (structure.getControl(0).equals(controls[6]) && structure.getControl(1).equals(controls[8]))
	            return 6;
	    } else if (structure.size() == 6 && structure.containSwitch(controls[9], controls[7])) {
	        if (structure.getControl(0).equals(controls[9]) && structure.getControl(1).equals(controls[7]))
	            return 7;
	        else if (structure.getControl(0).equals(controls[7]) && structure.getControl(1).equals(controls[11]))
	            return 8;
	        else if (structure.getControl(0).equals(controls[11]) && structure.getControl(1).equals(controls[7]))
	            return 9;
	        else if (structure.getControl(0).equals(controls[7]) && structure.getControl(1).equals(controls[13]))
	            return 10;
	        else if (structure.getControl(0).equals(controls[13]) && structure.getControl(1).equals(controls[7]))
	            return 11;
	        else if (structure.getControl(0).equals(controls[7]) && structure.getControl(1).equals(controls[9]))
	            return 12;
	    }
	    // Shuffle trajectory
	    else if (structure.containSwitch(controls[10], controls[13])) {
	        if (structure.getControl(0).equals(controls[10]) && structure.getControl(1).equals(controls[13]))
	            return 13;
	        else if (structure.getControl(0).equals(controls[13]) && structure.getControl(1).equals(controls[7]))
	            return 14;
	        else if (structure.getControl(0).equals(controls[7]) && structure.getControl(1).equals(controls[9]))
	            return 15;
	        else if (structure.getControl(0).equals(controls[9]) && structure.getControl(1).equals(controls[10]))
	            return 16;
	    } else if (structure.containSwitch(controls[13], controls[10])) {
	        if (structure.getControl(0).equals(controls[13]) && structure.getControl(1).equals(controls[10]))
	            return 17;
	        else if (structure.getControl(0).equals(controls[10]) && structure.getControl(1).equals(controls[6]))
	            return 18;
	        else if (structure.getControl(0).equals(controls[6]) && structure.getControl(1).equals(controls[8]))
	            return 19;
	        else if (structure.getControl(0).equals(controls[8]) && structure.getControl(1).equals(controls[13]))
	            return 20;
	    } else if (structure.containSwitch(controls[6], controls[12])) {
	        if (structure.getControl(0).equals(controls[6]) && structure.getControl(1).equals(controls[12]))
	            return 21;
	        else if (structure.getControl(0).equals(controls[12]) && structure.getControl(1).equals(controls[11]))
	            return 22;
	        else if (structure.getControl(0).equals(controls[11]) && structure.getControl(1).equals(controls[8]))
	            return 23;
	        else if (structure.getControl(0).equals(controls[8]) && structure.getControl(1).equals(controls[6]))
	            return 24;
	    } else if (structure.containSwitch(controls[7], controls[11])) {
	        if (structure.getControl(0).equals(controls[7]) && structure.getControl(1).equals(controls[11]))
	            return 25;
	        else if (structure.getControl(0).equals(controls[11]) && structure.getControl(1).equals(controls[12]))
	            return 26;
	        else if (structure.getControl(0).equals(controls[12]) && structure.getControl(1).equals(controls[9]))
	            return 27;
	        else if (structure.getControl(0).equals(controls[9]) && structure.getControl(1).equals(controls[7]))
	            return 28;
	    } else if (structure.containSwitch(controls[11], controls[7])) {
	        if (structure.getControl(0).equals(controls[11]) && structure.getControl(1).equals(controls[7]))
	            return 29;
	        else if (structure.getControl(0).equals(controls[7]) && structure.getControl(1).equals(controls[13]))
	            return 30;
	        else if (structure.getControl(0).equals(controls[13]) && structure.getControl(1).equals(controls[8]))
	            return 31;
	        else if (structure.getControl(0).equals(controls[8]) && structure.getControl(1).equals(controls[11]))
	            return 32;
	    } else if (structure.containSwitch(controls[12], controls[6])) {
	        if (structure.getControl(0).equals(controls[12]) && structure.getControl(1).equals(controls[6]))
	            return 33;
	        else if (structure.getControl(0).equals(controls[6]) && structure.getControl(1).equals(controls[10]))
	            return 34;
	        else if (structure.getControl(0).equals(controls[10]) && structure.getControl(1).equals(controls[9]))
	            return 35;
	        else if (structure.getControl(0).equals(controls[9]) && structure.getControl(1).equals(controls[12]))
	            return 36;
	    }
	    return Synthesis.NO_SOLUTION; // Cannot happen
	}
}
