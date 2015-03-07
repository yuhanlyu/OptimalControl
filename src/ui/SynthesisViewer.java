package ui;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;

import javax.swing.JPanel;


public class SynthesisViewer extends JPanel {
	private Drawing draw = Drawing.NULL_DRAWING;   // Draw the synthesis
	
	/**
	 * Constructor
	 */
	public SynthesisViewer() {
		this.addMouseMotionListener(new MouseMotionAdapter() {
			public void mouseMoved(MouseEvent e) {
				double x = (double)(e.getX() - getWidth()/2) / Viewer.SCALE;
				double y = -(double)(e.getY() - getHeight()/2) / Viewer.SCALE;
				System.out.println(x + " " + y);
			}
		});
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D)g;
		TrajectoryViewer.setRenderingHints(g2);
		g2.translate(this.getWidth()/2, this.getHeight()/2);
		g2.scale(1, -1);
		draw.draw(g2);
	}
	
	/**
	 * 
	 * @param draw
	 */
	public void setDrawing(Drawing draw) {
		this.draw = draw;
	}
	
	/**
	 * 
	 * @param type
	 */
	public void setType(int type) {
		draw.setType(type);
	}
}
