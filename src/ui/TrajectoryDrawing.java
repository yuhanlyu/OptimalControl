package ui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;

import optimalControl.Configuration;
import optimalControl.ControlLine;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import optimalControl.Utility;
import robotModel.Robot;

public class TrajectoryDrawing extends Drawing {
	
	private static final double DOT_SIZE = 0.2;
	private static final double MIN_X = -10;
	private static final double CONTROL_LINE_LENGTH = 100;
	private static final double ARROW_SIZE = 0.3;
	private static final Color green = new Color(0, 112, 60);
	private static final AffineTransform scale = AffineTransform.getScaleInstance(Viewer.SCALE, Viewer.SCALE);
	private static final Line2D orientation = new Line2D.Double(0, 0, 0.5, 0.0);
	private static final Path2D arrow = new Path2D.Double();
	static {
		arrow.moveTo(0,ARROW_SIZE);
		arrow.lineTo(0, -ARROW_SIZE);
		arrow.lineTo(Math.sqrt(3.0)*ARROW_SIZE/2, 0);
		arrow.closePath();
	}
	private Trajectory trajectory;
	private Robot robot;
	private ControlLine controlLine;
	private Transformation Ts;
	private double time = 0.0;
	private TrajectoryShape trajectoryShape;
	
	
	/**
	 * @param initial initial configuration
	 * @param robot the robot
	 * @param trajectory the trajectory
	 * @param L the control line
	 */
	public TrajectoryDrawing(Configuration qs, Robot robot, Trajectory trajectory, ControlLine controlLine) {
		this.robot = robot;
		this.trajectory = trajectory;
		this.controlLine = controlLine;
		this.Ts = new Transformation(qs);
		this.trajectoryShape = new TrajectoryShape(Ts, trajectory);
		System.out.println("Time is: " + trajectory.totalTime());
	}
	
	/**
	 * 
	 */
	@Override
	public void incrementTime(double step) {
		time += step;
		if (step > trajectory.totalTime())
			time = trajectory.totalTime();
		if (step == trajectory.totalTime())
			time = 0.0;
	}
	
	/**
	 * @param g the graphics to be drawn
	 * @param time the final time
	 */
	@Override
	public void draw(Graphics2D g) {
		Transformation T = Ts.move(trajectory, time);
		g.setStroke(new BasicStroke(3.0f));
		drawRobot(g, T);
		//drawRotationCenter(g, time);
		if (controlLine != null)
			drawControlLine(g);
		trajectoryShape.draw(g);
	}
	
	/**
	 * @param g the graphics to be drawn
	 * @param time the final time
	 */
	private void drawRotationCenter(Graphics2D g, double time) {
		Point2D rotationCenter = Utility.rotationCenter(Ts, trajectory, time);
		if (rotationCenter != null) {
			Ellipse2D rc = new Ellipse2D.Double(rotationCenter.getX()-DOT_SIZE/2, rotationCenter.getY()- DOT_SIZE/2, DOT_SIZE, DOT_SIZE);
			Color oldColor = g.getColor();
			g.setColor(Color.red);
			g.fill(scale.createTransformedShape(rc));
			g.setColor(oldColor);
		}
	}
	
	/**
	 * @param g the graphics to be drawn
	 * @param T the transformation of the configuration
	 */
	private void drawRobot(Graphics2D g, Transformation T) {
		AffineTransform trans = new AffineTransform(scale);			
		trans.translate(T.getX(), T.getY());
		trans.rotate(T.getTheta());
		g.draw(trans.createTransformedShape(robot.getShape()));
		g.draw(trans.createTransformedShape(orientation));
	}
	
	/**
	 * @param g the graphics to be drawn
	 */
	private void drawControlLine(Graphics2D g) {
		double b = -controlLine.getKtheta()/controlLine.getKx();
		Point2D p1 = new Point2D.Double(MIN_X * controlLine.getKx(), b + MIN_X * controlLine.getKy());
		Point2D p2 = new Point2D.Double(p1.getX() + CONTROL_LINE_LENGTH * controlLine.getKx(), p1.getY() + CONTROL_LINE_LENGTH * controlLine.getKy());
		Line2D cl = new Line2D.Double(p1, p2);
		
		AffineTransform trans = new AffineTransform(scale);
		trans.translate(p2.getX(), p2.getY());
		trans.rotate(Math.atan2(controlLine.getKy(), controlLine.getKx()));
		Color oldColor = g.getColor();
		g.setColor(green);
		g.draw(scale.createTransformedShape(cl));
		g.fill(trans.createTransformedShape(arrow));
		g.setColor(oldColor);
	}
}
