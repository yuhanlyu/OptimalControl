package ui;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JPanel;
import obstacleModel.Environment;

/**
 * Viewer for environments with obstales
 * @author yu-hanlyu
 *
 */
public class ObstacleViewer extends JPanel {
	private Environment env;
	private Drawing draw = Drawing.NULL_DRAWING;
	
	
	public ObstacleViewer(Environment env) {
		this.env = env;
	}
	
	public void setDrawing(Drawing draw) {
		this.draw = draw;
	}
	
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D)g;
		TrajectoryViewer.setRenderingHints(g2);
		g2.translate(this.getWidth()/2, this.getHeight()/2);
		g2.scale(1, -1);
		g2.setColor(Color.black);
	    draw.draw(g2);
	}
}
