package optimalControl;
import java.awt.geom.Point2D;

/**
 * Compute distance and time for a translation
 * @author yu-hanlyu
 *
 */
public class TranslationDTFunctor extends DTFunctor {
	private double d; // distance between sp(pre, u) and sp(next, u)
    private double sin; // angle between (rc(pre), sp(pre, u)) and (sp(next, u), sp(pre, u))
    private double cos;
    private double v; // velocity of the control
	
	public TranslationDTFunctor(Control pre, Control current, Control next) {
		Point2D preS = pre.switchPoint(current).toPoint();
		Point2D v2 = Utility.vector(preS, next.switchPoint(current).toPoint());
		double theta = Utility.angle(Utility.vector(preS, pre.rotationCenter()), v2, pre.getOmega() > 0);
		sin = Math.sin(theta);
		cos = Math.cos(theta);
		d = Utility.distanceToOrigin(v2);
		v = current.getVelocity();
	}

	@Override
	public DistanceTime getDistanceTime(double H) {
		double squareDiff = (v + H) * (v - H);
		if (squareDiff < 0)
        	throw new IllegalArgumentException("H not in range");
		double t = ((sin + (H * cos) / Math.sqrt(squareDiff)) * d) / v;
		return new DistanceTime(t * H, t);
	}

	@Override
	public DistanceTimeL getL(double H) {
		double squareDiff = (v + H) * (v - H);
		if (squareDiff < 0)
        	throw new IllegalArgumentException("H not in range");
        double timeL = (d * v * cos) / (Math.pow(squareDiff, 1.5));
        double t = ((sin + (H * cos) / Math.sqrt(squareDiff)) * d) / v;
        double distanceL = timeL * H + t;
        return new DistanceTimeL(distanceL, timeL);
	}
}
