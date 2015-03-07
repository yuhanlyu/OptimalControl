package ui;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Scanner;

import optimalControl.Configuration;
import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.Trajectory;
import robotModel.Robot;

/**
 * Read robot, trajectory from a file
 * @author yu-hanlyu
 *
 */
public class TrajectoryParser {
	private Robot robot;
	private Configuration initial;
	private ControlLine controlLine;
	private Trajectory trajectory;
	
	/**
	 * 
	 * @param fileName
	 */
	public TrajectoryParser(String fileName) {
		read(fileName);
	}
	
	/**
	 * 
	 * @param fileName
	 */
	private void read(String fileName) {
		try (BufferedReader inputFile =  new BufferedReader(new FileReader(fileName))) {
			robot = new Robot(null, readRobot(inputFile));
			initial = readInitial(inputFile);
			controlLine = readControlLine(inputFile);
			trajectory = readTrajectory(inputFile);
			
		} catch (FileNotFoundException e) {
			System.err.println("File not found: " + fileName);
			e.printStackTrace();
		} catch (IOException e) {
			System.err.println("IOException at read trajectory");
			e.printStackTrace();
		}
	}
	
	/**
	 * 
	 * @param inputFile
	 * @return
	 * @throws IOException
	 */
	private static Shape readRobot(BufferedReader inputFile) throws IOException {
		Path2D shape = new Path2D.Double();
		int numOfVertex = Integer.parseInt(inputFile.readLine());
		if (numOfVertex == 0) {
			return new Ellipse2D.Double(-0.5, -0.5, 1, 1);
		}
		try (Scanner scanner = new Scanner(inputFile.readLine())) {
			double x = scanner.nextDouble();
			double y = scanner.nextDouble();
			shape.moveTo(x, y);
		}
		for (int i = 1; i < numOfVertex; ++i) {
			try (Scanner scanner = new Scanner(inputFile.readLine())) {
				double x = scanner.nextDouble();
				double y = scanner.nextDouble();
				shape.lineTo(x, y);
			}
		}
		shape.closePath();
		return shape;
	}
	
	/**
	 * 
	 * @param inputFile
	 * @return
	 * @throws IOException
	 */
	private static Trajectory readTrajectory(BufferedReader inputFile) throws IOException {
		Trajectory traj = new Trajectory();
		int n = Integer.parseInt(inputFile.readLine());
		for (int i = 0; i < n; ++i) {
			try (Scanner scanner = new Scanner(inputFile.readLine())) {
				double vx = scanner.nextDouble();
				double vy = scanner.nextDouble();
				double omega = scanner.nextDouble();
				double time = scanner.nextDouble();
				traj.addControl(new Control(vx, vy, omega), time);
			}
		}
		return traj;
	}
	
	/**
	 * 
	 * @param inputFile
	 * @return
	 * @throws IOException
	 */
	private static Configuration readInitial(BufferedReader inputFile) throws IOException {
		try (Scanner scanner = new Scanner(inputFile.readLine())) {
			double x = scanner.nextDouble();
			double y = scanner.nextDouble();
			double theta = scanner.nextDouble();
			return new Configuration(x, y, theta);
		}
	}
	
	/**
	 * 
	 * @param inputFile
	 * @return
	 * @throws IOException
	 */
	private static ControlLine readControlLine(BufferedReader inputFile) throws IOException {
		try (Scanner scanner = new Scanner(inputFile.readLine())) {
			double kx = scanner.nextDouble();
			double ky = scanner.nextDouble();
			double ktheta = scanner.nextDouble();
			double H = scanner.nextDouble();
			return new ControlLine(kx, ky, ktheta, H);
		}
	}
	
	/**
	 * 
	 * @return
	 */
	public Robot getRobot() {
		return robot;
	}
	
	/**
	 * 
	 * @return
	 */
	public ControlLine getControlLine() {
		return controlLine;
	}
	
	/**
	 * 
	 * @return
	 */
	public Trajectory getTrajectory() {
		return trajectory;
	}
	
	/**
	 * 
	 * @return
	 */
	public Configuration getInitial() {
		return initial;
	}
}
