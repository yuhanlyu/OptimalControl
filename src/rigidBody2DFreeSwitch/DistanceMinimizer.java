package rigidBody2DFreeSwitch;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

import javax.imageio.ImageIO;

import optimalControl.Configuration;
import optimalControl.ControlLine;
import optimalControl.DistanceTime;
import optimalControl.Trajectory;
import robotModel.Robot;
import ui.TrajectoryDrawing;
import ui.TrajectoryViewer;
import ui.Viewer;

/**
 * Interface of solvers for minimization problems
 * @author yu-hanlyu
 *
 */
public abstract class DistanceMinimizer {
	protected double delta;      // The gap from the critical values
	protected double distError;  // The maximum distance error
	protected double timeError;  // The maximum time error
	
	/**
	 * Constructor
	 * @param delta the gap along boundaries
	 * @param distError maximum distance error
	 * @param timeError maximum time error
	 */
	public DistanceMinimizer(double delta, double distError, double timeError) {
		this.delta = delta;
		this.distError = distError;
		this.timeError = timeError;
	}
	
	/**
	 * This function is used for animation
	 * @param minimization
	 * @return
	 */
	public abstract GenericInfo minimize(DistanceMinimization minimization, List<Trajectory> trajList, List<ControlLine> controlLineList);
	
	/**
	 * 
	 * @param minimization
	 * @param findAllTrajectories
	 * @param trajectories
	 * @return
	 */
	public abstract GenericInfo minimize(DistanceMinimization minimization, boolean findAllTrajectories, List<TrajectoryInfo> trajectories);
	
	/**
	 * Minimize
	 * @param minimization
	 * @return
	 */
	public GenericInfo minimize(DistanceMinimization minimization) {
		return minimize(minimization, false, null);
	}
	
	/**
	 * Compare two pairs of distance and time
	 * @param lhs a pair of distance and time
	 * @param rhs a pair of distance and time
	 * @return negative if lhs is better than rhs
	 */
	public int compareDistanceTime(DistanceTime lhs, DistanceTime rhs) {
		if (lhs.getDistance() < distError && rhs.getDistance() < distError)
			return lhs.getTime() < rhs.getTime() ? -1 : 1;
		return lhs.getDistance() < rhs.getDistance() ? -1 : 1;
	}
	
	/**
	 * Compare two solutions
	 * @param lhs a solution
	 * @param rhs a solution
	 * @return negative if lhs is better than rhs
	 */
	public int compareSolution(GenericInfo lhs, GenericInfo rhs) {
		return compareDistanceTime(lhs.getDistanceTime(), rhs.getDistanceTime());
	}
	
	/**
	 * 
	 */
	public void saveFile(Configuration qs, Robot robot, List<Trajectory> trajList, List<ControlLine> controlLineList) {
		for (int i = 0; i < trajList.size(); ++i) {
			Trajectory traj = trajList.get(i);
			ControlLine controlLine = controlLineList.get(i);
			TrajectoryDrawing draw = new TrajectoryDrawing(qs, robot, traj, controlLine);
			
			
			BufferedImage image = new BufferedImage(Viewer.VIEWER_WIDTH, Viewer.VIEWER_HEIGHT, BufferedImage.TYPE_INT_ARGB_PRE);
			Graphics2D g2 = image.createGraphics();
			TrajectoryViewer.setRenderingHints(g2);
			g2.setColor(Color.white);
			g2.fillRect(0,  0, Viewer.VIEWER_WIDTH, Viewer.VIEWER_HEIGHT);
			g2.setColor(Color.black);
			g2.translate(Viewer.VIEWER_WIDTH/2,Viewer.VIEWER_HEIGHT/2);
			g2.scale(1, -1);
			
			draw.draw(g2);
			File outputfile = new File("images/movie" + String.format("%03d", i) + ".png");
			try {
				ImageIO.write(image, "png", outputfile);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
