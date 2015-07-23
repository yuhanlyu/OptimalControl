package rigidBody2DFreeSwitch;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.logging.Logger;
import java.util.stream.IntStream;

import optimalControl.Configuration;
import optimalControl.ControlSet;
import optimalControl.Transformation;
import robotModel.Robot;

/**
 * Creating synthesis
 * @author yu-hanlyu
 *
 */
public class Synthesis {
	private static final Logger logger = Logger.getLogger(Synthesis.class.getName());
	public static final int NO_SOLUTION = -1;
	public static final int FEASIBLE_SOLUTION = -2;
	public static final int TGT_SOLUTION = -3;
	public static final int WHIRL_SOLUTION = -4;
	public static final int SINGULAR_SOLUTION = -5;
	public static final int GENERIC_SOLUTION = 1;
	private static final boolean PARALLEL = false;
	private Robot robot;
	private double xLowerBound, xUpperBound;
	private double yLowerBound, yUpperBound;
	private double theta;
	private double resolution;
	private int xLength;
	private int yLength;
	private int [][] grid;
	
	/**
	 * Constructor
	 * @param builder
	 */
	private Synthesis(SynthesisBuilder builder) {
		this.robot = builder.robot;
		this.xLowerBound = builder.xLowerBound;
		this.xUpperBound = builder.xUpperBound;
		this.yLowerBound = builder.yLowerBound;
		this.yUpperBound = builder.yUpperBound;
		this.theta = builder.theta;
		this.resolution = builder.resolution;
		synthesis();
	}
	
	/**
	 * Create a mapping from points to trajectory types
	 * @return a mapping from points to trajectory types
	 */
	public HashMap<Point2D.Double, Integer> getTrajectoryMapping() {
		HashMap<Point2D.Double, Integer> result = new HashMap<>();
		for (int xIndex = 0; xIndex < xLength; ++xIndex) {
			for (int yIndex = 0; yIndex < yLength; ++yIndex) {
				double x = xLowerBound + xIndex * resolution;
				double y = yLowerBound + yIndex * resolution;
				result.put(new Point2D.Double(x, y), grid[yIndex][xIndex]);
			}
		}
		return result;
	}
	
	/**
	 * Create the whole synthesis
	 */
	private void synthesis() {
		logger.info("Start to compute synthesis");
		xLength = (int)((xUpperBound - xLowerBound) / resolution) + 1;
		yLength = (int)((yUpperBound - yLowerBound) / resolution) + 1;
		grid = new int[yLength][xLength];
		long startTime = System.nanoTime();
		if (PARALLEL)
			IntStream.range(0, yLength).parallel().forEach(y -> synthesis(y));
		else
			IntStream.range(0, yLength).forEach(y -> synthesis(y));
		long endTime = System.nanoTime();
		double duration = (endTime - startTime) / 1000000000.0;
		double rates = (xLength * yLength) / duration;
		logger.info("Configurations per second is " + rates);
	}
	
	/**
	 * Create synthesis for a specific y value
	 * @param y y-coordinate
	 */
	private void synthesis(int y) {
		IntStream.range(0, xLength).forEach(x -> synthesis(x, y));
	}
	
	/**
	 * Create synthesis for specific x value and y value
	 * @param x x-coordinate
	 * @param y y-coordinate
	 */
	private void synthesis(int xIndex, int yIndex) {
		double x = xLowerBound + xIndex * resolution;
		double y = yLowerBound + yIndex * resolution;
		logger.info("Configuraiton: " + x + " " + y + " " + theta);
		ControlSet U = robot.getControlSet();
		Configuration qs = new Configuration(x, y, theta);
		Transformation Ts = new Transformation(qs);
		TrajectoryInfo solution = new FreePlanner().solve(U, Ts);
		if (solution == null) {
			grid[yIndex][xIndex] = Synthesis.NO_SOLUTION;
		} else {
			grid[yIndex][xIndex] = robot.analyzeTrajectory(solution);
		}
	}
	
	/**
	 * Builder for synthesis
	 * @author yu-hanlyu
	 *
	 */
	public static class SynthesisBuilder {
		private Robot robot;
		private double xLowerBound, xUpperBound;
		private double yLowerBound, yUpperBound;
		private double theta;
		private double resolution;
		
		/**
		 * Constructor
		 * @param robot
		 */
		public SynthesisBuilder(Robot robot) {
			this.robot = robot;
		}
		
		/**
		 * Set the lower and upper bounds for the x-coordinate
		 * @param xLowerBound
		 * @param xUpperBound
		 * @return
		 */
		public SynthesisBuilder setX(double xLowerBound, double xUpperBound) {
			this.xLowerBound = xLowerBound;
			this.xUpperBound = xUpperBound;
			return this;
		}
		
		/**
		 * Set the lower and upper bound for the y-coordinate
		 * @param yLowerBound
		 * @param yUpperBound
		 * @return
		 */
		public SynthesisBuilder setY(double yLowerBound, double yUpperBound) {
			this.yLowerBound = yLowerBound;
			this.yUpperBound = yUpperBound;
			return this;
		}
		
		/**
		 * Set the theta value
		 * @param theta
		 * @return
		 */
		public SynthesisBuilder setTheta(double theta) {
			this.theta = theta;
			return this;
		}
		
		/**
		 * Set the resolution
		 * @param resolution
		 * @return
		 */
		public SynthesisBuilder setResolution(double resolution) {
			this.resolution = resolution;
			return this;
		}
		
		/**
		 * build
		 * @return
		 */
		public Synthesis build() {
			return new Synthesis(this);
		}
	}
}
