package ui;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import optimalControl.Interval;

import org.junit.Test;

import rigidBody2DFreeSwitch.Synthesis;
import robotModel.OmniDrive;
import robotModel.Robot;


public class SynthesisAnimate {
	private static final Interval rangeX = new Interval(-2, 2);
	private static final Interval rangeY = new Interval(-2, 2);
	private static final double RESOLUTION = 0.1;
	private static final double STEP = Math.PI / 15;

	@Test
	public void test() {
		System.out.println("Dubins Car");
		//testRobot(new DubinsCar());
		System.out.println("Reed-Shepp Car");
		// testRobot(new RSCar()); 
		System.out.println("Omnidirectional Vehicle");
		testRobot(new OmniDrive());
		System.out.println("Differential Drive");
		//testRobot(new DiffDrive());
	}
	
	public static void testRobot(Robot robot) {
		for (int i = 0; i < 30; ++i) {
			testSynthesis(robot, i * STEP, i);
		}
	}
	
	public static void testSynthesis(Robot robot, double theta, int count) {
		System.out.println(count);
		Synthesis synthesis = new Synthesis.SynthesisBuilder(robot).setX(rangeX.getBegin(), rangeX.getEnd())
                                           .setY(rangeY.getBegin(), rangeY.getEnd())
                                           .setTheta(theta)
                                           .setResolution(RESOLUTION)
                                           .build();
		SynthesisDrawing draw = new SynthesisDrawing(synthesis);
		BufferedImage image = new BufferedImage(Viewer.VIEWER_WIDTH, Viewer.VIEWER_HEIGHT, BufferedImage.TYPE_INT_ARGB_PRE);
		Graphics2D g2 = image.createGraphics();
		
		TrajectoryViewer.setRenderingHints(g2);
		g2.setColor(Color.white);
		g2.fillRect(0,  0, Viewer.VIEWER_WIDTH, Viewer.VIEWER_HEIGHT);
		g2.setColor(Color.black);
		g2.translate(Viewer.VIEWER_WIDTH/2,Viewer.VIEWER_HEIGHT/2);
		g2.scale(3, -3);
		
		draw.draw(g2);
		File outputfile = new File("images/movie" + String.format("%03d", count) + ".png");
		try {
			ImageIO.write(image, "png", outputfile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
