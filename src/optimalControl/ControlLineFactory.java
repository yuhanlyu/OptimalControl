package optimalControl;
import java.awt.geom.Point2D;

public class ControlLineFactory {
	private double maxa, maxb, maxg; // Store the control with the largest absolute angular velocity
	private ControlLineFunctor functor; // Functor for building control line
	private Interval range = Interval.EMPTY_INTERVAL;
	private Control us;
	private Control uf;
	
	/**
	 * Create a control line factory based on initial configuration, first control, final configuration and final
	 * control
	 * @param Ts initial configuration
	 * @param us first control
	 * @param Tf final configuration
	 * @param uf final control
	 */
	public ControlLineFactory(ControlSet U, Transformation Ts, Control us, Transformation Tf, Control uf, boolean isPositive) {
		this.us = us;
		this.uf = uf;
	    Homogeneous cws = Ts.transform(new Homogeneous(us)), cwf = Tf.transform(new Homogeneous(uf));
	    
	    double as = cws.getX(), bs = cws.getY(), gs = cws.getOmega();
	    double af = cwf.getX(), bf = cwf.getY(), gf = cwf.getOmega();
	    double a = as - af, b = bs - bf, g = gs - gf;
	    
	    // Initial control and final control have the same velocity
	    // Then, we can shift from one to another.
	    // W.L.O.G, we can assume one of them has zero time
	    // Hence, we don't need to consider this case.
	    if (Utility.isZero(a) && Utility.isZero(b) && Utility.isZero(g)) {
	    	functor = new NullLineFunctor();
	    	return;
	    }
	    
	    if (Math.abs(us.getOmega()) > Math.abs(uf.getOmega())) {
	    	maxa = as;
	    	maxb = bs;
	    	maxg = gs;
	    } else {
	    	maxa = af;
	    	maxb = bf;
	    	maxg = gf;
	    }
	    functor = Utility.isZero(g) ? new EqualOmegaFunctor(a, b, isPositive) : 
	    		                      new UnequalOmegaFunctor(a, b, g, af, bf, gf, isPositive);
	    //RangeFinder rangeFinder = new RangeFinder(U, Ts, us, Tf, uf, this);
	    //positiveRange = rangeFinder.getRange(true);
	    //negativeRange = rangeFinder.getRange(false);
	    range = new RangeFinder(U, Ts, us, Tf, uf, this).getRange();
	}
	
	/**
	 * Return the range
	 * @return the range
	 */
	public Interval getRange() {
		return range;
	}
	
	/**
	 * Return the first control
	 * @return the first control
	 */
	public Control getUs() {
		return us;
	}
	
	/**
	 * Return the last control
	 * @return the last control
	 */
	public Control getUf() {
		return uf;
	}
	
	/**
	 * Compute the Hamiltonian value that u and ui have the same Hamiltonian at T
	 * @param T trnasformation in the world frame
	 * @param u control
	 * @param ui control
	 * @return Hamiltonian value that u and ui have the same Hamiltonian at TLR
	 *         the computation only based on the formula, hence H may lead to an invalid control line
	 *         or may be negative. Return NaN when there is no solution
	 */
	public double switchPointOnLine(Transformation T, Control u, Control ui) {
		return functor.switchPointOnLine(T, u, ui);
	}
	
	/**
	 * Return the upper bound of the Hamiltonian for the generation of control lines
	 * @return the upper bound of the Hamiltonian
	 */
	public double getUpperBoundOfH() {
		return functor.getUpperBoundOfH();
	}
	
	/**
	 * For a given H value and sign, return the corresponding control line 
	 * @param H the Hamiltonian value
	 * @param isRounding whether to round result matching H, should be used in singular trajectory
	 * @return a control line corresponds to the H value and the sign
	 */
	public ControlLine getControlLine(double H, boolean isRounding) {
		return functor.getControlLine(H, isRounding);
	}
	
	/**
	 * For a given H value and sign, return the corresponding control line without rounding
	 * @param H the Hamiltonian value
	 * @return a control line corresponds to the H value and the sign
	 */
	public ControlLine getControlLine(double H) {
		return functor.getControlLine(H, false);
	}
	
	/**
	 * Compute Lipschitz constants of kx, ky, and ktheta with respect to H
	 * @param H the Hamiltonian value
	 * @return Lipschitz constants
	 */
	public ControlLineL getControlLineL(double H) {
		return functor.getControlLineL(H);
	}
	
	/**
	 * Compute the derivate of x and y with respect to H
	 * @param H the Hamiltonian value
	 * @param T the configuration
	 * @return an array with length 4 storing dx, dy, dcos, dsin
	 */
	public PositionL getMappingL(double H, Transformation T) {
		ControlLineL controlLineL = functor.getControlLineL(H);
		double kxL = controlLineL.getKxL();
		double kyL = controlLineL.getKyL();
		double kthetaL = controlLineL.getKthetaL();
		double xL = kxL * Math.abs(T.getX()) + kyL * Math.abs(T.getY());
		double yL = kyL * Math.abs(T.getX()) + kxL * Math.abs(T.getY()) + kthetaL;
		double cosL = kxL * Math.abs(T.getCos()) + kyL * Math.abs(T.getSin());
		double sinL = kxL * Math.abs(T.getSin()) + kyL * Math.abs(T.getCos());
		return new PositionL(xL, yL, cosL, sinL);
	}
	
	/**
	 * Returning the functor to compute Lipschitz constant
	 * @param T a configuration of interest
	 * @param u current control
	 * @param next next control
	 * @return LFunctor
	 */
	public LFunctor getLFunctor(Transformation T, Control u, Control next) {
		return u.isTranslation() ? new TranslationFunctor(T, u, next) 
		                         : new RotationFunctor(T, u, next);
	}
	
	/**
	 * Compute ktheta value based on kx, ky, and H
	 * use maxa, maxb, maxg to reduce numerical unstabillity
	 * @param kx kx
	 * @param ky ky
	 * @param H Hamiltonian value
	 * @return ktheta
	 */
	private double computeKtheta(double kx, double ky, double H) {
		return (H + ky * maxa - kx * maxb) / maxg;
	}
	
	/**
	 * 
	 * @author yu-hanlyu
	 *
	 */
	private abstract class ControlLineFunctor {
		
		/**
		 * Compute a control line with respect to H and positve
		 * @param H H must be positive
		 * @return a control line
		 */
		public abstract ControlLine getControlLine(double H, boolean isRounding);
		
		/**
		 * Compute the Hamiltonian value that u and ui have the same Hamiltonian at T
		 * @param T trnasformation in the world frame
		 * @param u control
		 * @param ui control
		 * @return Hamiltonian value that u and ui have the same Hamiltonian at TLR
		 *         Return NaN when there is no solution for H > 0
		 */
		public abstract double switchPointOnLine(Transformation T, Control u, Control ui);
		
		/**
		 * Return the upper bound of the Hamiltonian for the generation of control lines
		 * @return the upper bound of the Hamiltonian
		 */
		public abstract double getUpperBoundOfH();
		
		/**
		 * Return the Lipschitz constants of k's with respect to H
		 * @param H the Hamiltonian value
		 * @return Lipschitz constants
		 */
		public abstract ControlLineL getControlLineL(double H);
		
		/**
		 * Compute the Lipschitz constant of ktheta based on the Lipschitz of kx, ky and H
		 * @param kxL the Lipschitz constant of kx
		 * @param kyL the Lipschitz constant of ky
		 * @param H the Hamiltonian value
		 * @return the Lipschitz constant of dktheta
		 */
		public double kthetaL(double kxL, double kyL, double H) {
			return (1 + Math.abs(maxa) * kyL + Math.abs(maxb) * kxL) / Math.abs(maxg);
		}
		
		/**
		 * Test whether the y-coordinate transforming SP attach to T in the world frame 
		 * to the control line frame is zero
		 * @param T transformation in the world frame
		 * @param SP a homogeneous point attatched to the robot
		 * @param controlLine control line
		 * @return true, if the y-coordinate is zero, otherwise false
		 */
		public boolean isOnLine(Transformation T, Homogeneous SP, ControlLine controlLine) {
			return Utility.isZero((new Transformation(controlLine)).transform(T.transform(SP)).getY());
		}
	}
	
	private class NullLineFunctor extends ControlLineFunctor {
		
		@Override
		public ControlLine getControlLine(double H, boolean isRounding){ 
			return ControlLine.NULL_LINE; 
		}

		@Override
		public double switchPointOnLine(Transformation T, Control u, Control ui) {
			return 0;
		}
		
		@Override
		public double getUpperBoundOfH() { return 0; }

		@Override
		public ControlLineL getControlLineL(double H) {
			return null;
		}
	}
	
	/**
	 * Functor for building control line in the case of initial control and last control have the same
	 * angular velocity
	 * @author yu-hanlyu
	 *
	 */
	private class EqualOmegaFunctor extends ControlLineFunctor {
		private double r, kx, ky;
		
		/**
		 * Constructor stores necessary information
		 * @param a
		 * @param b
		 * @param isPositive the sign of the control line
		 */
		public EqualOmegaFunctor(double a, double b, boolean isPositive) {
			r = Math.sqrt(a * a + b * b);
			kx = a / r;
			ky = b / r;
			if (!isPositive) {
				kx = -kx;
				ky = -ky;
			}
		}
	
		@Override
		public ControlLine getControlLine(double H, boolean isRounding){
			return new ControlLine(kx, ky, computeKtheta(kx, ky, H), H);
		}
		
		@Override
		public double switchPointOnLine(Transformation T, Control u, Control ui) {
			// There is no solution for this case
			if (Utility.absEqual(u.getOmega(), ui.getOmega()))
				return Double.NaN;
			Homogeneous SP = ui.switchPoint(u);
			Point2D p = T.transform(SP).toPoint();
			double H = ky * (maxg * p.getX() - maxa) - kx * (maxg * p.getY() - maxb);
			return H >= 0.0 ? H : Double.NaN;
		}
		
		@Override
		public double getUpperBoundOfH() { 
			return r;
		}

		@Override
		public ControlLineL getControlLineL(double H) {
			return new ControlLineL(0, 0, kthetaL(0, 0, H), 0);
		}
	}
	
	/**
	 * Functor for building control line in the case of initial control and last control have different
	 * angular velocities
	 * @author yu-hanlyu
	 *
	 */
	private class UnequalOmegaFunctor extends ControlLineFunctor {
		private double rsquare, ap, bp;
		private double a, b, g, r;
		private int m;
		
		/**
		 * Constructor stores necessary information
		 * @param a
		 * @param b
		 * @param g
		 * @param af
		 * @param bf
		 * @param gf
		 * @param isPositive the sign of the control line
		 */
		public UnequalOmegaFunctor(double a, double b, double g, double af, double bf, double gf, boolean isPositive) {
			ap = af - (a * gf) / g;
			bp = bf - (b * gf) / g;
			rsquare = ap * ap + bp * bp;
			r = Math.sqrt(rsquare);
			m = isPositive ? 1 : -1;
			this.a = a;
			this.b = b;
			this.g = g;
		}
		
		@Override
		public ControlLine getControlLine(double H, boolean isRounding) {
			if (isRounding && Utility.absEqual(r, H))
				H = r;
			if (r < Math.abs(H))
				return ControlLine.NULL_LINE;
			double exp = m * Math.sqrt(rsquare - H * H);
			double k1 = (bp * H + ap * exp) / rsquare;
			double k2 = (bp * exp - ap * H) / rsquare;
			return new ControlLine(k1, k2, computeKtheta(k1, k2, H), H);
		}
		
		
		@Override
		public double switchPointOnLine(Transformation T, Control u, Control ui) {
			double ph;
			Homogeneous SP = ui.switchPoint(u);
			
			if (Utility.isZero(SP.getOmega())) {
				Homogeneous q = T.transform(SP);
				ph = Math.atan2(q.getY(), q.getX());
			} else {
				Point2D q = T.transform(SP).toPoint();
				ph = Math.atan2(g * q.getY() - b, g * q.getX() - a);
			}
			double H = r * Math.abs(Math.cos(ph + Math.atan2(ap, bp)));
			// Solution is one of H and -H. Verify the solution!
			return isOnLine(T, SP, getControlLine(H, false)) ? H : Double.NaN;
		}
		
		@Override
		public double getUpperBoundOfH() { 
			return r;
		}

		@Override
		public ControlLineL getControlLineL(double H) {
			double sqrtSquareDiff = Math.sqrt(rsquare - H * H);
			double kxL = (Math.abs(bp) + Math.abs((ap * H) / sqrtSquareDiff)) / rsquare;
			double kyL = (Math.abs(ap) + Math.abs((bp * H) / sqrtSquareDiff)) / rsquare;
			return new ControlLineL(kxL, kyL, kthetaL(kxL, kyL, H), 1 / sqrtSquareDiff);
		}
	}
	
	/**
	 * The Lipschitz constant functor
	 * @author yu-hanlyu
	 *
	 */
	public interface LFunctor {
		public DistanceTimeL getL(double H);
	}
	
	/**
	 * A functor for computing Lipschitz constant for translation
	 * @author yu-hanlyu
	 *
	 */
	private class TranslationFunctor implements LFunctor {
		private Transformation P; // The position of the switch point in the world frame
		private Control uW;       // The velocity in the world frame
		private Transformation T; // A configuration in interest
		
		/**
		 * Constructor
		 * @param T
		 * @param u
		 * @param next
		 */
		public TranslationFunctor(Transformation T, Control u, Control next) {
			Point2D SP = T.transform(u.switchPoint(next)).toPoint();
			P = new Transformation(new Configuration(SP.getX(), SP.getY(), 0));
			uW = u.toWorld(T);
			this.T = T;
		}
		
		@Override
		public DistanceTimeL getL(double H) {
			ControlLine controlLine = getControlLine(H);
			Transformation TLW = new Transformation(controlLine);
			Control uL = uW.toWorld(TLW); // Velocity in the control line frame
			Point2D SPL = TLW.transform(P).toPoint(); // Switch point int the control line frame
			double t = SPL.getY() / -uL.getVy(); // duration for the control
			
			ControlLineL controlLineL = getControlLineL(H);
			double kxL = controlLineL.getKxL();
			double kyL = controlLineL.getKyL();
			double kthetaL = controlLineL.getKthetaL();
			double vyL = uL.getVy();
			double PyL = SPL.getY();
			double dvyL = Math.abs(kyL * uW.getVx()) + Math.abs(kxL * uW.getVy());
			double dvxL = Math.abs(kxL * uW.getVx()) + Math.abs(kyL * uW.getVy());
			double dPyL = Math.abs(kyL * P.getX()) + Math.abs(kxL * P.getY()) + kthetaL;
			double timeL = (Math.abs(dPyL * vyL) + Math.abs(PyL * dvyL)) / (vyL * vyL);
			
			PositionL positilnL = getMappingL(H, T);
			double distL = Math.abs(uL.getVx() * timeL) + Math.abs(dvxL * t) + positilnL.getXL();
			//double distL = Math.abs(uL.getVx() * timeL) + Math.abs(dvxL * t); // Probably don't need getXL
			if (!Utility.absEqual(uL.getVx(), H)) {
				//System.out.println("Error in getL " + H + " " + uL.getVx());
			}
			return new DistanceTimeL(distL, timeL);
		}
	}
	
	/**
	 * A functor for computing Lipschitz constant for Rotation
	 * @author yu-hanlyu
	 *
	 */
	private class RotationFunctor implements LFunctor {
		private Transformation RCW;  // Rotation center in the world frame
		private boolean sameOmega;   // Whether the control and the next have the same angular velocity
		private Transformation T;    // A configuration in interest
		private double r;
		private double d;
		private double sin;
		private double cos;
		private double omega;
		private double omegaD;

		public RotationFunctor(Transformation T, Control u, Control next) {
			RCW = new Transformation(u.rotationCenter(T));
			sameOmega = u.getOmega() == next.getOmega();
			Point2D SP = u.switchPoint(next).toPoint();
			Point2D c = u.rotationCenter();
			r = Utility.distanceToOrigin(c);
			d = SP.distance(c);
			double theta = Utility.angle(Utility.vector(c, SP), Utility.vector(c, new Point2D.Double()), true);
			sin = Math.abs(Math.sin(theta));
			cos = Math.abs(Math.cos(theta));
			omega = Math.abs(u.getOmega());
			omegaD = omega * d;
			this.T = T;
		}

		@Override
		public DistanceTimeL getL(double H) {
			PositionL startPositionL = getMappingL(H, T);
			PositionL rcPositionL = getMappingL(H, RCW);
			double distL = rcPositionL.getXL() + startPositionL.getXL();
			double timeL = getControlLineL(H).getPhiL();
			if (!sameOmega) {
				double sqrtSquareDiff = Math.sqrt((omegaD + H) * (omegaD - H));
				distL += ((sin + (cos * H) / sqrtSquareDiff) * r) / omegaD;
				//timeL += (omegaD / sqrtSquareDiff); // Fix at 10/26
				timeL += (1.0 / sqrtSquareDiff);
			}
			return new DistanceTimeL(distL, timeL / omega);
		}
	}
	
	/**
	 * A class storing Lipschitz constants for the control line
	 * @author yu-hanlyu
	 *
	 */
	public static class ControlLineL {
		private double kxL;
		private double kyL;
		private double kthetaL;
		private double phiL; // The Lipschitz constant for the angle of the control line
		
		/**
		 * Constructor
		 * @param kxL
		 * @param kyL
		 * @param kthetaL
		 */
		public ControlLineL(double kxL, double kyL, double kthetaL, double phiL) {
			this.kxL = kxL;
			this.kyL = kyL;
			this.kthetaL = kthetaL;
			this.phiL = phiL;
		}
		
		/**
		 * Return the upper-bound of the Lipschitz constant of kx
		 * @return
		 */
		public double getKxL() {
			return kxL;
		}
		
		/**
		 * Return the upper-bound of the Lipschitz constant of ky
		 * @return
		 */
		public double getKyL() {
			return kyL;
		}
		
		/**
		 * Return the upper-bound of the Lipschitz constant of ktheta
		 * @return
		 */
		public double getKthetaL() {
			return kthetaL;
		}
		
		/**
		 * Return the upper-bound of the Lipschitz constant of phi
		 * @return
		 */
		public double getPhiL() {
			return phiL;
		}
	}
	
	/**
	 * A class storing Lipschitz constants for the mapping from a configuration to the control line
	 * @author yu-hanlyu
	 *
	 */
	public static class PositionL {
		private double xL;
		private double yL;
		private double cosL;
		private double sinL;
		
		/**
		 * Constructor
		 * @param xL
		 * @param yL
		 * @param cosL
		 * @param sinL
		 */
		public PositionL(double xL, double yL, double cosL, double sinL) {
			this.xL = xL;
			this.yL = yL;
			this.cosL = cosL;
			this.sinL = sinL;
		}
		
		/**
		 * Return the Lipschitz constant of x-coordinate
		 * @return
		 */
		public double getXL() {
			return xL;
		}
		
		/**
		 * Return the Lipschitz constant of y-coordinate
		 * @return
		 */
		public double getYL() {
			return yL;
		}
		
		/**
		 * Return the Lipschitz constant of cos theta
		 * @return
		 */
		public double getCosL() {
			return cosL;
		}
		
		/**
		 * Return the Lipschitz constant of sin theta
		 * @return
		 */
		public double getSinL() {
			return sinL;
		}
	}
	
	private class MaxFunctor implements LFunctor {
		private DistanceTimeL mappingL;
		private Interval range;
		
		public MaxFunctor(Transformation Ts, Transformation Tf, Interval range) {
			this.range = range;
		}
		
		@Override
		public DistanceTimeL getL(double H) {
			return null;
		}
	}
}
