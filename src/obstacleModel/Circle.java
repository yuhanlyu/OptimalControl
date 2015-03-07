package obstacleModel;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import optimalControl.Control;
import optimalControl.Transformation;
import optimalControl.Utility;


public class Circle extends Obstacle {
	private Point2D center;
	private double radius;
	
	public Circle(Point2D center, double radius) {
		super(new Ellipse2D.Double(center.getX() - radius, center.getY() - radius, radius * 2, radius * 2));
		this.center = center;
		this.radius = radius;
	}
	
	@Override
	protected double translateToWall(Transformation T, Control u) {
		Point2D p = T.toPoint();
		Control worldU = u.toWorld(T);
		
		double x = p.getX();
		double y = p.getY();
		double cx = center.getX();
		double cy = center.getY();
		double vx = worldU.getVx();
		double vy = worldU.getVy();
		
		// Set up a quadratic formula
		double a = vx * vx + vy * vy;
		double b = 2 * (vx * (x - cx) + vy * (y - cy));
		double c = (x - cx) * (x - cx) + (y - cy) * (y - cy) - radius * radius;
		double discriminant = b * b - 4 * a * c;
		
		// If no intersection
		if (discriminant < 0) {
			return Double.POSITIVE_INFINITY;
		}
		// If inside the circle
		if (c * a < 0) {
			return 0;
		}
		// If both solutions are negative
		if (a * b > 0) {
			return Double.POSITIVE_INFINITY;
		}
		
		// Avoid catastrophic cancellation
		double root1;
		if (b > 0) {
			root1 = ((-b - Math.sqrt(discriminant)) * 0.5) / a;
		} else {
			root1 = ((-b + Math.sqrt(discriminant))  * 0.5) / a;
		}
		double root2 = c / (a * root1);
		return Math.min(root1, root2);
	}
	
	@Override
	protected double rotateToWall(Transformation T, Control u) {
		// Find solutions of (x-a)^2 + (y-b)^2 = r0^2, (x-c)^2 + (y-d)^2 = r1^2
		double r0 = radius;
		double a = center.getX();
		double b = center.getY();
		Point2D rc = u.rotationCenter(T);
		Point2D p = T.toPoint();
		double r1 = rc.distance(p);
		double c = p.getX();
		double d = p.getY();
		
		double D = rc.distance(center);
		if (D > r0 + r1 || D < Math.abs(r0 - r1))
			return Double.POSITIVE_INFINITY;
		double expr = Math.sqrt((D + r0 + r1) * (D + r0 - r1) * (D - r0 + r1) * (-D +r0 + r1)) * 0.25;
		
		// The intersection points are (x1, y1), (x2, y2)
		double x1 = (a + c) * 0.5 + ((c - a) * (r0 * r0 - r1 * r1) * 0.5 + 2 * (b - d) * expr) / (D * D);
		double x2 = (a + c) * 0.5 + ((c - a) * (r0 * r0 - r1 * r1) * 0.5 - 2 * (b - d) * expr) / (D * D);
		double y1 = (b + d) * 0.5 + ((d - b) * (r0 * r0 - r1 * r1) * 0.5 - 2 * (a - c) * expr) / (D * D);
		double y2 = (b + d) * 0.5 + ((d - b) * (r0 * r0 - r1 * r1) * 0.5 + 2 * (a - c) * expr) / (D * D);
		
		// Compute the time to collide and return the minimum
		double theta0 = Math.atan2(p.getY() - rc.getY(), p.getX() - rc.getX());
		double theta1 = Math.atan2(y1 - rc.getY(), x1 - rc.getX());
		double theta2 = Math.atan2(y2 - rc.getY(), x2 - rc.getX());
		double t1 = Utility.timeToAngle(theta0, theta1, u.getOmega());
		double t2 = Utility.timeToAngle(theta0, theta2, u.getOmega());
		return Math.min(t1, t2);
	}
	
	@Override
	public boolean contains(Point2D p) {
		return onBorder(p) ? true : (center.distance(p) < radius);
	}

	private boolean onBorder(Point2D p) {
		return Utility.absEqual(center.distance(p), radius);
	}

	@Override
	public double[] derivative(Point2D p) {
		System.out.println(p.getX() + " " + center.getX());
		return new double[]{2.0 * (p.getX() - center.getX()),
				            2.0 * (p.getY() - center.getY())};
	}

	@Override
	public List<Transformation> generateSamples(double delta) {
		List<Transformation> result = new ArrayList<>();
		int n = (int)Math.ceil(2.0 * Math.PI * radius / delta);
		double step = 2.0 * Math.PI / n;
		for (int i = 0; i < n; ++i) {
			double angle = step * i;
			double x = center.getX() + radius * Math.cos(angle);
			double y = center.getY() + radius * Math.sin(angle);
			for (double thetaStep = 0; thetaStep < Math.PI; thetaStep += Math.PI / 3.0) {
				double theta = Utility.normalize(angle + thetaStep);
				double cos = -Math.sin(theta);
				double sin = Math.cos(theta);
				result.add(new Transformation(x, y, sin, cos));
				result.add(new Transformation(x, y, -sin, -cos));
			}
		}
		return result;
	}

	private boolean followBorder(Transformation T, Control u) {
		if (u.isTranslation())
			return false;
		Point2D rc = u.rotationCenter(T);
		if (Utility.isZero(rc.distance(center)))
			return true;
		return false;
	}

	private boolean leaveBorder(Transformation T, Control u) {
		if (u.isTranslation()) {
			Point2D p = T.toPoint();
			Point2D centerToP = Utility.vector(center, p);
			return Utility.isZero(centerToP.getX() * T.getCos() + centerToP.getY() * T.getSin());
		}
		Point2D rc = u.rotationCenter(T);
		double distance = center.distance(rc);
		if (Utility.absEqual(distance, radius + u.radius()))
			return true;
		return false;
	}

	@Override
	public boolean isCollide(Transformation T, Control u, double duration) {
		if (!onBorder(T.toPoint())) {
			double timeToWall = timeToWall(T, u);
			return timeToWall < duration ? true : false;
		}
		if (followBorder(T, u))
			return false;
		if (leaveBorder(T, u))
			return false;
		return true;
	}
}
