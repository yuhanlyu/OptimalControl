package ui;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JPanel;
import javax.swing.Timer;


public class AnimateViewer extends JPanel {
	private static final double STEP = 0.1; // The step size in the animation
	private Drawing draw = Drawing.NULL_DRAWING;
	
	public AnimateViewer() {
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
	 * @param initial initial configuration
	 * @param robot the robot
	 * @param trajectory the trajectory
	 * @param L the control line
	 */
	public void startAnimate() {
		new Timer(100, new ActionListener(){
		    @Override
			public void actionPerformed(ActionEvent evt){
		    	draw.incrementTime(STEP);
		        repaint();
		    }
		}).start();
	}
}
