package optimalControl;
import java.awt.geom.Point2D;

/**
 * Computing distance and time for a rotation
 * @author yu-hanlyu
 *
 */
public class RotationDTFunctor extends DTFunctor {
	private double thetarc1;
    private double thetarc2;
    private double pred;
    private double nextd;
    private double omega;
    private double r;
    private double theta;
    private int m1; // multiplier one
    private int m2; // multiplier two
    private static final double m[] = new double[]{-1.0, 1.0};
	
    /**
     * Constructor
     * @param pre previous control
     * @param current current control
     * @param next next control
     */
	public RotationDTFunctor(Control pre, Control current, Control next) {
		double omegaDiff1 = current.getOmega() - pre.getOmega();
        double omegaDiff2 = current.getOmega() - next.getOmega();
        Point2D preS = new Homogeneous(pre.getVy() - current.getVy(), 
        		                       current.getVx() - pre.getVx(),
        		                       Utility.isZero(omegaDiff1) ? 1.0 : omegaDiff1).toPoint();
        		
        Point2D nextS = new Homogeneous(next.getVy() - current.getVy(),
        		                        current.getVx() - next.getVx(),
        		                        Utility.isZero(omegaDiff2) ? 1.0 : omegaDiff2).toPoint();
        
        Point2D c = current.rotationCenter();
        /**
         * v1 is the vector from c to nextS
         * v2 is the vector from c to preS
         * v3 is the vector from c to origin
         */
        Point2D v1 = Utility.vector(c, nextS);
        Point2D v2 = Utility.vector(c, preS);
        Point2D v3 = Utility.vector(c, new Point2D.Double());

        thetarc1 = Utility.angle(v2, v3, true);
        //thetarc2(angle(v1, v3, true)),
        //use this formula can avoid problem when v3 = (0.0, 0.0)
        thetarc2 = Utility.angle(v1, v2, true) + thetarc1;
        theta = Utility.angle(v1, v2, true);
        pred = Utility.isZero(omegaDiff1) ? Double.POSITIVE_INFINITY : Utility.distanceToOrigin(v2);
        nextd = Utility.isZero(omegaDiff2) ? Double.POSITIVE_INFINITY : Utility.distanceToOrigin(v1);
        omega = current.getOmega();
        r = Utility.distanceToOrigin(c);
        m1 = (current.getOmega() > 0.0) != (omegaDiff1 < 0.0) ? 1 : 0;
        m2 = (current.getOmega() > 0.0) != (omegaDiff2 < 0.0) ? 0 : 1;
	}

	@Override
	public DistanceTime getDistanceTime(double H) {
		double thetaPre = Math.acos(H/(omega * pred));
        double thetaNext = Math.acos(H/(omega * nextd));
        if (Double.isNaN(thetaPre) || Double.isNaN(thetaNext))
        	throw new IllegalArgumentException("H not in range");

        // theta2 - theta1
        double angleDiff = m[m2] * thetaNext - m[m1] * thetaPre + theta;
        // theta2 + theta1 + M_PI
        double angleSum = m[m2] * thetaNext + m[m1] * thetaPre + theta
                          + 2.0 * thetarc1;
        return new DistanceTime(2.0 * r * Math.cos(angleSum * 0.5) * 
                                          Math.sin(angleDiff * 0.5),
                                Utility.timeOfRotateTheta(angleDiff, omega));
	}

	@Override
	public DistanceTimeL getL(double H) {
		double exp1 = omega * pred;
		double preSquareDiff = (exp1 + H) * (exp1 - H);
		double exp2 = omega * nextd;
		double nextSquareDiff = (exp2 + H) * (exp2 - H);
        if (preSquareDiff < 0 || nextSquareDiff < 0)
        	throw new IllegalArgumentException("H not in range");
        double timeL = Math.abs((1.0 / Math.sqrt(preSquareDiff) + 1.0 / Math.sqrt(nextSquareDiff)) / omega);
        double term1 = Math.abs(Math.sin(thetarc1)) 
                     + Math.abs((H * Math.cos(thetarc1)) / 
                                 Math.sqrt(preSquareDiff));
        double term2 = Math.abs(Math.sin(thetarc2))
                     + Math.abs((H * Math.cos(thetarc2)) /
                                 Math.sqrt(nextSquareDiff));
        double distL = Math.abs((r * ((term1 / pred) + (term2 / nextd))) / omega);
        
        return new DistanceTimeL(distL, timeL);
	}
}
