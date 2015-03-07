package obstacleModel;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import optimalControl.Control;
import optimalControl.Trajectory;
import optimalControl.Transformation;

/**
 * A class storing obstacles
 * @author yu-hanlyu
 *
 */
public class Environment {
	private List<Obstacle> obstacles = new ArrayList<>();
	private List<Shape> shapes = new ArrayList<>();
	
	public void addObstacle(Obstacle o) {
		obstacles.add(o);
		shapes.add(o.getShape());
	}
	
	public List<Shape> getShapes() {
		return shapes;
	}
	
	public static double[] getDerivative(Point2D p) {
		return null;
	}
	
	public boolean isCollide(Transformation T, Control u, double duration) {
		for (Obstacle o : obstacles) {
			if (o.isCollide(T, u, duration))
				return true;
		}
		return false;
	}
	
	public boolean isCollide(Transformation T, Trajectory trajectory) {
		for (int i = 0; i < trajectory.size(); ++i) {
			Control u = trajectory.getControl(i);
			double duration = trajectory.getDuration(i);
			if (isCollide(T, u, duration))
				return true;
			T = T.move(trajectory.getControl(i), trajectory.getDuration(i));
		}
		return false;
	}
	
	public List<List<Transformation>> generateSamples(double delta) {
		List<List<Transformation>> result = new ArrayList<>();
		for (Obstacle o : obstacles)
			result.add(o.generateSamples(delta));
		return result;
	}
}
