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
public class Bench extends Robot{
	private static final Shape shape = new Rectangle2D.Double(-0.1, 1, 0.2, 2);
	private static final Control[] controls = new Control[]{new Control(1, 0, 1), 
		                                                    new Control(1, 0, -1),
		                                                    new Control(-1, 0, 1),
		                                                    new Control(-1, 0, -1)};
	private static final List<Double> critHs = Arrays.asList(new Double(1.0));
	
	/**
	 * Constructor
	 */
	public Bench() {
		super(new ControlSet(Arrays.asList(controls), critHs, 1), shape);
	}
	
	@Override
	protected int analyzeGeneric(ControlStructure structure) {
	    return Synthesis.NO_SOLUTION; // Cannot happen
	}
}
