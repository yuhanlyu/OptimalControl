package ui;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;


public class TrajectoryViewer extends JPanel {
	private Drawing draw = Drawing.NULL_DRAWING;                   // Draw the trajectory
	
	/**
	 * 
	 */
	public TrajectoryViewer() {
	}
	
	public void saveImage() {
		BufferedImage bi = new BufferedImage(this.getWidth(), this.getHeight(), BufferedImage.TYPE_INT_RGB);
		
		this.paintComponent(bi.getGraphics());
	    File outputfile = new File("data/saved.png");
	    try {
			ImageIO.write(bi, "png", outputfile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * 
	 * @param draw
	 */
	public void setDrawing(Drawing draw) {
		this.draw = draw;
	}
	
	/* (non-Javadoc)
	 * @see javax.swing.JComponent#paintComponent(java.awt.Graphics)
	 */
	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D)g;
		TrajectoryViewer.setRenderingHints(g2);
		g.setColor(Color.WHITE);  
		g.fillRect(0, 0, this.getWidth(), this.getHeight());  
		g2.setColor(Color.black);
		g2.translate(this.getWidth()/2, this.getHeight()/2);
		g2.scale(1.8, -1.8);
		draw.draw(g2);
	}
	
	/**
	 * @param g enable all renderhints to enhance the quality
	 */
	public static void setRenderingHints(Graphics2D g) {
		g.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION, RenderingHints.VALUE_ALPHA_INTERPOLATION_QUALITY);
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_COLOR_RENDERING, RenderingHints.VALUE_COLOR_RENDER_QUALITY);
		g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BICUBIC);		
	}
}
