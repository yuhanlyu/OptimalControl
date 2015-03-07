package ui;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import rigidBody2DFreeSwitch.Synthesis;

public class SynthesisDrawing extends Drawing {
	private static final double DOT_SIZE = 0.05;
	private static final int SCALE = Viewer.SCALE;
	private static final AffineTransform scale = AffineTransform.getScaleInstance(SCALE, SCALE);
	private HashMap<Integer, Color> colorMap;
	private HashMap<Point2D.Double, Integer> synthesis;
	
	/**
	 * Constructor
	 * @param synthesis
	 */
	public SynthesisDrawing(Synthesis synthesis) {
		this.synthesis = synthesis.getTrajectoryMapping();
	}
	
	/**
	 * Draw the synthesis on the graphics
	 * @param g
	 */
	@Override
	public void draw(Graphics2D g) {
		colorMapping();
		Color oldColor = g.getColor();
		for (Point2D p : synthesis.keySet()) {
			int solutionType = synthesis.get(p);
			if (type != Drawing.UNFILTER && (type < Synthesis.GENERIC_SOLUTION) && type != solutionType)
				continue;
			if (type == Synthesis.GENERIC_SOLUTION && solutionType < Synthesis.GENERIC_SOLUTION)
				continue;
			g.setColor(colorMap.get(synthesis.get(p)));
			Ellipse2D circle = new Ellipse2D.Double(p.getX() - DOT_SIZE/2, p.getY() - DOT_SIZE/2, DOT_SIZE, DOT_SIZE);
			g.fill(scale.createTransformedShape(circle));
		}
		g.setColor(oldColor);
	}
	
	/**
	 * Create the color mapping
	 */
	private void colorMapping() {
		colorMap = new HashMap<>();
		Set<Integer> types = new HashSet<>(); 
	    types.addAll(synthesis.values());
	    int N = types.size();
	    float step = 0.3f / N;
	    float hue = 0.3f;
	    for (Integer i : types) {
	    	if (i == Synthesis.NO_SOLUTION)
	    		colorMap.put(i, Color.WHITE);
	    	else if (i == Synthesis.SINGULAR_SOLUTION)
	    		colorMap.put(i, Color.BLACK);
	    	else if (i == Synthesis.TGT_SOLUTION)
	    		colorMap.put(i, Color.getHSBColor(0.0f, 1.0f, 1.0f));
	    	else if (i == Synthesis.WHIRL_SOLUTION)
	    		colorMap.put(i, Color.getHSBColor(0.1f, 1.0f, 1.0f));
	    	else if (i == Synthesis.FEASIBLE_SOLUTION)
	    		colorMap.put(i, Color.getHSBColor(0.2f, 1.0f, 1.0f));
	    	else
	    		colorMap.put(i, Color.getHSBColor(hue, 1.0f, 1.0f));
	    	hue += step;
	    }
	}
}
