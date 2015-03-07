package obstacleModel;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.util.List;

import optimalControl.Control;
import optimalControl.Transformation;

/**
 * An interface for obstacle
 * @author yu-hanlyu
 *
 */
public abstract class Obstacle {
	private Shape shape;
	
	public Obstacle(Shape shape) {
		this.shape = shape;
	}
	
	/**
	 * Compute the minimum time for a control from T to the obstacle
	 * @param T a configuration
	 * @param u a translation
	 * @return time
	 */
	public final double timeToWall(Transformation T, Control u) {
		if (contains(T.toPoint()))
			return 0;
		return u.isTranslation() ? translateToWall(T, u) : rotateToWall(T, u);
	}
	
	/**
	 * Compute the minimum time for a translation from T to the obstacle
	 * @param T a configuration
	 * @param u a translation
	 * @return time
	 */
	protected abstract double translateToWall(Transformation T, Control u);
	
	/**
	 * Compute the minimum time for a rotation from T to the obstacle
	 * @param T a configuration 
	 * @param u a rotation
	 * @return time
	 */
	protected abstract double rotateToWall(Transformation T, Control u);
	
	public abstract boolean isCollide(Transformation T, Control u, double time);
	
	public abstract boolean contains(Point2D p);
	
	public abstract double[] derivative(Point2D p);
	
	public final Shape getShape() { return shape; }
	
	public abstract List<Transformation> generateSamples(double delta);
}
