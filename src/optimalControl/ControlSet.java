package optimalControl;
import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * @author yu-hanlyu
 *
 */
public class ControlSet implements Iterable<Control> {
	private List<Control> controls; // Stores all controls
	private List<Double> critHs;
	private double upperBound;
	
	/**
	 * Constructor 
	 * @param arg_controls all controls in the set
	 */
	public ControlSet(List<Control> controls, List<Double> critHs, double upperBound) {
		this.controls = new ArrayList<>(controls);
		this.critHs = new ArrayList<>(critHs);
		this.upperBound = upperBound;
	}
	
	/**
	 * Create the reverse of the control set
	 * @return the reverse of the control set
	 */
	public ControlSet reverse() {
		List<Control> reverseControls = controls.stream()
				                                .map(u -> u.reverse())
				                                .collect(Collectors.toList());
		return new ControlSet(reverseControls, critHs, upperBound);
	}
	
	/**
	 * @return the number of controls
	 */
	public int size() {
		return controls.size();
	}
	
	/**
	 * @param index
	 * @return the i-th control
	 */
	public Control getControl(int index) {
		return controls.get(index);
	}
	
	/**
	 * Return stream of controls
	 * @return a stream of controls
	 */
	public Stream<Control> controlStream() {
		return controls.stream();
	}
	
	@Override
	public Iterator<Control> iterator() {
		return controls.iterator();
	}
	
	/**
	 * 
	 * @return the number of critical values for this control set
	 */
	public int getCriticalSize() {
		return critHs.size();
	}
	
	/**
	 * 
	 * @param index
	 * @return the index-th critical value
	 */
	public double getCriticalValue(int index) {
		return critHs.get(index);
	}
	
	/**
	 * Return stream of critical values
	 * @return a stream of critical values
	 */
	public Stream<Double> criticalValueStream() {
		return critHs.stream();
	}
	
	/**
	 * Return the upper bound of the Hamiltonian for non-trivial trajectories
	 * @return the upper bound
	 */
	public double getUpperBound() {
		return upperBound;
	}
	
	/**
	 * Determine whether control u is the only sustainable control at configuration TLR 
	 * in the control line frame
	 * @param TLR configuration in the control line frame
	 * @param u control
	 * @return true is u is sustainable at configuration TLR
	 */
	public boolean isSustainable(Transformation TLR, Control u) {
		List<Control> sustainableControls = sustainableControls(TLR);
		return sustainableControls.size() == 1 && sustainableControls.contains(u);
	}
	
	/**
	 * Determine whether control u is the one of the controls that maximize H at configuration TLR 
	 * in the control line frame
	 * @return true if u is maximizing H
	 */
	public boolean isMaximizing(Transformation TLR, Control u) {
		return maximizingControls(TLR).contains(u);
	}
	
	/**
	 * 6.3.1 equation (6.17)
	 * This is different from Furtuna's thesis
	 * @param ui a control
	 * @param u another control
	 * @param TLR a configuration in the control line frame
	 * @return the derivative of H of u with respect to ui, larger than zero means that ui will have higher H value
	 */
	private static double dHamiltonian(Transformation TLR, Control ui, Control u) {
		Control dq = u.toWorld(TLR), uiT = ui.toWorld(TLR);
	    return dq.getVy() * uiT.getOmega() - uiT.getVy() * dq.getOmega();
	}
	
	/**
	 * Compute the Hamiltonian when apply u at TLR in the control line frame
	 * @return the Hamiltonian
	 */
	public static double Hamiltonian(Transformation TLR, Control u) {
	    return TLR.transform(new Homogeneous(u)).getY();
	}

	/**
	 * Compute the set of controls with approximate maximum H value at TLR in the control line frame
	 * @param TLR a configuration in the control line frame
	 * @return the list of controls with approximate maximum H value
	 */
	private List<Control> maximizingControls(Transformation TLR) {
		// Finding the maximum H value
		double max = controls.stream().mapToDouble(u -> Hamiltonian(TLR, u)).max().getAsDouble();
		// Collect the set of controls with approximate maximum H value
		return controls.stream().filter(u -> Utility.absEqual(Hamiltonian(TLR, u), max)).collect(Collectors.toList());
	}
	
	/**
	 * Compute the set of sustainable controls at TLR in the control line frame
	 * @param TLR a configuration in the control line frame
	 * @return the list of sustainable controls
	 */
	public List<Control> sustainableControls(Transformation TLR) {
		List<Control> maxUs = maximizingControls(TLR);
		if (maxUs.size() == 1)
			return maxUs;
		
		// a control is sustainable if it is maximizing and its dHi is negative with respect to other maximizing controls
		return maxUs.stream()
				    .filter(i -> maxUs.stream()
				     		          .filter(j -> dHamiltonian(TLR, j, i) > 0.0 && !Utility.isZero(dHamiltonian(TLR, j, i)))
				    		          .count() == 0)
				    .collect(Collectors.toList());
	}
	
	/**
	 * Return the sustainable control in the control line frame
	 * @param TLR a configuration in the control line frame
	 * @return the sustainable control
	 */
	public Control sustainableControl(Transformation TLR) {
		List<Control> sustainableControls = sustainableControls(TLR);
		if (sustainableControls.size() > 1)
			throw new IllegalArgumentException("Not generic segment");
		return sustainableControls.get(0);
	}
	
	/**
	 * Return the next sustainable control in the control line frame
	 * @param TLR a configuration in the control line frame
	 * @return the sustainable control
	 */
	public Control nextSustainableControl(Transformation TLR) {
		ControlTime controlTime = getGenericSegment(TLR);
		return sustainableControl(TLR.move(controlTime));
	}
	
	/**
	 * Return the next sustainable control in the control line frame assuming u is the current maximizing control
	 * @param TLR a configuration in the control line frame
	 * @return the next sustainable control
	 */
	public Control nextSustainableControl(Transformation TLR, Control u) {
		double time = switchTime(TLR, u, 0.0);
		return sustainableControl(TLR.move(u, time));
	}
	
	/**
	 * Compute the time of applying u until switch at configuration TLR with minimum duration minTime
	 * @param TLR configuration
	 * @param u control
	 * @param minTime minimum duration
	 * @parm round whether want to round
	 * @return time of applying u until switch
	 */
	public double switchTime(Transformation TLR, Control u, double minTime, boolean round) {
	    return controls.stream()
	    		       .mapToDouble(i -> u.equals(i) ? Double.POSITIVE_INFINITY : timeToSwitch(TLR, u, i, round))
	    		       .reduce(Double.POSITIVE_INFINITY, (double min, double switchTime) -> (switchTime > minTime && switchTime < min) ? switchTime : min);
	}
	
	/**
	 * Compute the time of applying u until switch at configuration TLR with minimum duration minTime
	 * @param TLR configuration
	 * @param u control
	 * @param minTime minimum duration
	 * @return time of applying u until switch
	 */
	public double switchTime(Transformation TLR, Control u, double minTime) {
	    return controls.stream()
	    		       .mapToDouble(i -> u.equals(i) ? Double.POSITIVE_INFINITY : timeToSwitch(TLR, u, i, false))
	    		       .reduce(Double.POSITIVE_INFINITY, (double min, double switchTime) -> (switchTime >= minTime && switchTime < min) ? switchTime : min);
	}
	
	/**
	 * Compute the time of applying u1 until switching to u2 at configuration TLR
	 * [thesis Section 6.3.3]
	 * @param TLR configuration
	 * @param u1 current control
	 * @param u2 next control
	 * @return duration of u1
	 */
	public static double timeToSwitch(Transformation TLR, Control u1, Control u2, boolean round) {
		return timeToNullY(TLR.transform(u1.switchPoint(u2)),
                           TLR.transform(new Homogeneous(u1)),
                           round);
	}
	
	/**
	 * Applying control R until the switch point P collides the line
	 * We want an answer strictly larger than zero
	 * If P is already is on x-axis, find the smallest t > 0, such that
	 * P collides with the x-axis
	 * [thesis Section 6.3.3, page 53]
	 * @param P switch point represented as a homogeneous point
	 * @param R control represented as a homogeneous point
	 * @return time before collide
	 */
	public static double timeToNullY(Homogeneous P, Homogeneous R, boolean round) {
	    // Degenerate case: P is already on the x-axis
	    //if (is_zero(P.y()))
	    //    return 0;
	    // Degenerate case: P and R are the same
	    if ((Utility.isZero(P.getOmega()) && Utility.isZero(R.getOmega())) || P.samePoint(R)) {
	        return Double.POSITIVE_INFINITY;
	    }
	    // Translation R.omega() == 0
	    if (Utility.isZero(R.getOmega())) {
	        if (Utility.isZero(R.getX())) {
	            return Double.POSITIVE_INFINITY;
	        }
	        double t = (P.getY() / P.getOmega()) / R.getX();
	        return t > 0.0 && !Utility.isZero(t) ? t : Double.POSITIVE_INFINITY;
	    }
	    // Rotation
	    return minrot(P, R, round);
	}
	
	/**
	 * R must be a rotation
	 * [thesis Section 6.3.3]
	 * @param P switch point represented as a homogeneous point
	 * @param R control represented as a homogeneous point
	 * @return time before collide
	 */
	private static double minrot(Homogeneous P, Homogeneous R, boolean round) {
	    double b1 = P.getX() - (P.getOmega() * R.getX()) / R.getOmega();
	    double b2 = P.getY() - (P.getOmega() * R.getY()) / R.getOmega();
	    double b3 = (P.getOmega() * R.getY()) / R.getOmega();
	    double r = Math.sqrt(b1 * b1 + b2 * b2);
	    return timeToCollideC2(-Math.atan2(b1, b2), -b3 / r, R.getOmega(), round);
	}	
	
	/**
	 * Section 6.3.2
	 * Solving min t > 0 that cos(tw + c1) = c2, assuming |c2| <= 1.0
	 * General form for t is (-c1 + acos(c2) + 2pi * c) / w or
	 *                       (-c1 - acos(c2) + 2pi * c) / w
	 * Pick the minimum of t > 0
	 * The following code is different from thesis, but this one
	 * is more accurate in terms of computation
	 * Using modulous is more accurate than using floor, since we can
	 * avoid division
	 * @param c1
	 * @param c2
	 * @param omega
	 * @return
	 */
	private static double timeToCollideC2(double c1, double c2, double omega, boolean round) {
		// These conditions will destroy stability, but is necessary for singular
		if (round) {
			if (c2 > 1.0 && Utility.absEqual(c2, 1.0))
		        c2 = 1.0;
		    else if (c2 < -1.0 && Utility.absEqual(c2, -1.0))
		        c2 = -1.0;
		}
	    if (Math.abs(c2) > 1.0)
	        return Double.POSITIVE_INFINITY;
	    double ac = Math.acos(c2);
	    double t1 = (-c1 - ac) % (2.0 * Math.PI);
	    double t2 = (-c1 + ac) % (2.0 * Math.PI);
	    // Adjust for the opposite sign, so that w and t1, t2 have the same sign
	    if (omega * t1 < 0.0)
	        t1 += (omega > 0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (omega * t2 < 0.0)
	        t2 += (omega > 0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (t1 == 0)
	    	t1 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (t2 == 0)
	    	t2 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (round) {
	    	if (Utility.isZero(t1))
	    		t1 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    	if (Utility.isZero(t2))
	    		t2 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    }
	    return Double.min(t1 / omega, t2 / omega);
	}
	
	/**
	 * Compute a sustainable control with its duration at TLR
	 * @param TLR configuration in the control line frame
	 * @return a control and its duration
	 */
	public ControlTime getGenericSegment(Transformation TLR) {
	    Control u = sustainableControl(TLR);
	    return new ControlTime(u, switchTime(TLR, u, 0.0));
	}
	
	/**
	 * Compute a sustainable control with its duration at TLR
	 * @param TLR configuration in the control line frame
	 * @param round whether want to round 
	 * @return a control and its duration
	 */
	public ControlTime getGenericSegment(Transformation TLR, boolean round) {
	    Control u = sustainableControl(TLR);
	    return new ControlTime(u, switchTime(TLR, u, 0.0, round));
	}
	
	/**
	 * Compute n generic segments at TLR
	 * @param TLR configuration in the control line frame
	 * @param n the length of generic segments
	 * @return a trajectory containing n generic segments
	 */
	/*
	public Trajectory getGenericSegment(Transformation TLR, int n) {
		Trajectory result = new Trajectory();
	    for (int i = 0; i < n; ++i) {
	        ControlTime controlTime = getGenericSegment(TLR);
	        result.addControl(controlTime.getControl(), controlTime.getTime());
	        TLR = TLR.move(controlTime);
	    }
	    return result;
	}*/
	
	/**
	 * Compute a period of generic segments at TLR 
	 * Assuming the first control is not too short 
	 * This assumption may not be true when you start in the middle of the action or the action is short 
	 * @param TLR configuration in the control line frame
	 * @return a trajectory containing a period of generic segments
	 */
	public Trajectory getPeriod(Transformation TLR) {
		Trajectory period = new Trajectory();
		while (true) {
			ControlTime ct = getGenericSegment(TLR, true);
	        if (period.size() == 1 && ct.getControl().equals(period.getControl(0))) {
	            break;
	        } else 
	        	if (period.size() >= 3 
	                && ct.getControl().equals(period.getControl(1))
	                && period.getControl(0).equals(period.getControl(period.size() - 1))) {
	            period.removeControl();
	            break;
	        }
	        TLR = TLR.move(ct);
	        if (period.size() >= 1 && ct.getControl().equals(period.getControl(period.size() - 1))) {
	        	ct = new ControlTime(ct.getControl(), ct.getTime() + period.getDuration(period.size()-1));
	        	period.removeControl();
	        } 
	        period.addControl(ct.getControl(), ct.getTime());
	    }
		return period;
	}
	
	/**
	 * Compute a period of generic segments at TLR 
	 * Assuming the first control is not too short 
	 * If the first control equals the previous control, then will left shift
	 * This assumption may not be true when you start in the middle of the action or the action is short 
	 * @param TLR configuration in the control line frame
	 * @pre previous control
	 * @return a trajectory containing a period of generic segments
	 */
	public Trajectory getPeriod(Transformation TLR, Control pre) {
		Trajectory period = getPeriod(TLR);
		if (period.getControl(0).equals(pre)) {
			period.leftRotate();
		}
		return period;
	}
	
	/**
	 * Find a generic trajectory from T to a singular point starting with control u
	 * @param TLR
	 * @return a result trajectory, or null if failed
	 */
	public Trajectory getExcursion(Transformation TLR, Control us, double minTime) {
		double ts = this.switchTime(TLR, us, minTime, true);
		if (Double.isInfinite(ts))
			return null;
		Trajectory excursion = new Trajectory();
		excursion.addControl(us, ts);
		TLR = TLR.move(us, ts);
		
	    for (int i = 0; i < size() * size() + 1; ++i) {
	    	List<Control> sustainableControls = sustainableControls(TLR);
	    	if (sustainableControls.size() > 1) {
	            return excursion;
	        }
	    	Control u = sustainableControls.get(0);
	    	double time = switchTime(TLR, u, minTime, true);
	    	if (Double.isInfinite(time))
	    		return null;
	    	excursion.addControl(u, time);
	        TLR = TLR.move(u, time);
	    }
	    return null;
	}
	
	/**
	 * Find a generic trajectory from T to a singular point starting
	 * @param TLR a configuration in the control line frame
	 * @return a result trajectory, or null if failed
	 */
	public Trajectory getExcursion(Transformation TLR, double minTime) {
		Trajectory excursion = new Trajectory();
	    for (int i = 0; i < size() * size() + 1; ++i) {
	    	List<Control> sustainableControls = sustainableControls(TLR);
	    	if (sustainableControls.size() > 1) {
	            return excursion;
	        }
	    	Control u = sustainableControls.get(0);
	    	double time = switchTime(TLR, u, minTime, true);
	    	if (Double.isInfinite(time))
	    		return null;
	    	excursion.addControl(u, time);
	        TLR = TLR.move(u, time);
	    }
	    return null;
	}
	
	/**
	 * Complete the current segment assuming u is the maximizing control and the next control is after
	 * @param TLR the configuration in the control line frame
	 * @param u current maximizing control
	 * @param afterU next maximizing control
	 * @return a pair of transformation and time, where transformation is the first configuration that
	 * applying u until both u and after are maximizing and time is the duration of u in order to reach this
	 * configuration
	 */
	public static TransformationTime completeSegment(Transformation TLR, Control u, Control after) {
		double time = timeToSwitch(TLR, u, after, false);
		if (Double.isInfinite(time))
			return new TransformationTime(TLR, Double.POSITIVE_INFINITY);
		return new TransformationTime(TLR.move(u, time), time);
	}
	
	/**
	 * Complete the current segment assuming u is the maximizing control
	 * @param TLR TLR the configuration in the control line frame
	 * @param u current maximizing control
	 * @return a pair of transformation and time, where transformation is the first configuration that
	 * applying u until both u and after are maximizing and time is the duration of u in order to reach this
	 * configuration
	 */
	public TransformationTime completeSegment(Transformation TLR, Control u) {
		double time = switchTime(TLR, u, 0.0);
		return new TransformationTime(TLR.move(u, time), time);
	}
	
	/**
	 * Determine whether there exists a translation parallel to the control line from TLR 
	 * @param TLR a configuration in the control line frame
	 * @param H Hamiltonian value
	 * @return a feasible translation, null otherwise
	 */
	public Control singularTranslation(Transformation TLR, double H) {
		for (Control u : controls) {
			if (u.isRotation())
				continue;
			Control v = u.toWorld(TLR);
			if (Utility.isZero(v.getVy()) && Utility.absEqual(v.getVx(), H))
				return u;
		}
		return null;
	}
	
	/**
	 * Return the maximum velocity
	 * @return maximum velocity
	 */
	public double maxVelocity() {
		return controls.stream().mapToDouble(u -> u.getVelocity()).max().getAsDouble();
	}
	
	/**
	 * Two durations version of timeToCollideC2, see timeToCollideC2
	 * @param c1
	 * @param c2
	 * @param omega
	 * @param round
	 * @return
	 */
	private static double[] timeToCollideC2_2(double c1, double c2, double omega, boolean round) {
		// These conditions will destroy stability, but is necessary for singular
		if (round) {
			if (c2 > 1.0 && Utility.absEqual(c2, 1.0))
		        c2 = 1.0;
		    else if (c2 < -1.0 && Utility.absEqual(c2, -1.0))
		        c2 = -1.0;
		}
	    if (Math.abs(c2) > 1.0)
	    	return new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
	    double ac = Math.acos(c2);
	    double t1 = (-c1 - ac) % (2.0 * Math.PI);
	    double t2 = (-c1 + ac) % (2.0 * Math.PI);
	    // Adjust for the opposite sign, so that w and t1, t2 have the same sign
	    if (omega * t1 < 0.0)
	        t1 += (omega > 0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (omega * t2 < 0.0)
	        t2 += (omega > 0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (t1 == 0)
	    	t1 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (t2 == 0)
	    	t2 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    if (round) {
	    	if (Utility.isZero(t1))
	    		t1 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    	if (Utility.isZero(t2))
	    		t2 = (omega > 0.0 ? 2.0 * Math.PI : -2.0 * Math.PI);
	    }
	    t1 /= omega;
	    t2 /= omega;
	    if (t1 > t2) {
	    	double temp = t1;
	    	t1 = t2;
	    	t2 = temp;
	    }
	    if (t1 == t2)
	    	t2 = Double.POSITIVE_INFINITY;
	    return new double[]{t1, t2};
	}
	
	/**
	 * Two durations version of minrot, see minrot
	 * @param P
	 * @param R
	 * @param round
	 * @return
	 */
	private static double[] minrot2(Homogeneous P, Homogeneous R, boolean round) {
	    double b1 = P.getX() - (P.getOmega() * R.getX()) / R.getOmega();
	    double b2 = P.getY() - (P.getOmega() * R.getY()) / R.getOmega();
	    double b3 = (P.getOmega() * R.getY()) / R.getOmega();
	    double r = Math.sqrt(b1 * b1 + b2 * b2);
	    return timeToCollideC2_2(-Math.atan2(b1, b2), -b3 / r, R.getOmega(), round);
	}
	
	/**
	 * Two durations version of timeToNullY, see timeToNullY
	 * @param P
	 * @param R
	 * @param round
	 * @return
	 */
	public static double[] timeToNullY2(Homogeneous P, Homogeneous R, boolean round) {
	    // Degenerate case: P is already on the x-axis
	    //if (is_zero(P.y()))
	    //    return 0;
	    // Degenerate case: P and R are the same
	    if ((Utility.isZero(P.getOmega()) && Utility.isZero(R.getOmega())) || P.samePoint(R)) {
	        return new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
	    }
	    // Translation R.omega() == 0
	    if (Utility.isZero(R.getOmega())) {
	        if (Utility.isZero(R.getX())) {
	        	return new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
	        }
	        double t = (P.getY() / P.getOmega()) / R.getX();
	        if (t > 0.0 && !Utility.isZero(t))
	        	return new double[]{t, Double.POSITIVE_INFINITY};
	        return new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
	    }
	    // Rotation
	    return minrot2(P, R, round);
	}
	
	/**
	 * Two durations of switching time, see timeToSwitch
	 * @param TLR
	 * @param u1
	 * @param u2
	 * @param round
	 * @return
	 */
	public static double[] timeToSwitch2(Transformation TLR, Control u1, Control u2, boolean round) {
		return timeToNullY2(TLR.transform(u1.switchPoint(u2)),
                           TLR.transform(new Homogeneous(u1)),
                           round);
	}
	
	/**
	 * Complete the segment at TLR applying u until switch to next
	 * @param TLR a Configuration
	 * @param u current control
	 * @param next next control
	 * @param round rounding mode
	 * @return
	 */
	public static TransformationTime[] completeSegment2(Transformation TLR, Control u, Control next, boolean round) {
		double[] duration = timeToSwitch2(TLR, u, next, round);
		Transformation T1 = null, T2 = null;
		if (Double.isFinite(duration[0])) {
			T1 = TLR.move(u, duration[0]);
		}
		if (Double.isFinite(duration[1])) {
			T2 = TLR.move(u, duration[1]);
		}
		return new TransformationTime[] {new TransformationTime(T1, duration[0]),
				                         new TransformationTime(T2, duration[1])};
	}
	
	/**
	 * Complete the segment at TLR applying u until switch to next
	 * @param TLR a configuration
	 * @param u current control
	 * @param next next control
	 * @return two possible durations of switching time
	 */
	public static TransformationTime[] completeSegment2(Transformation TLR, Control u, Control next) {
		return completeSegment2(TLR, u, next, false);
	}

	/**
	 * Complete the segment at TLR applying the reverse of u until switch to the reverse of previous
	 * @param TLR a configuration
	 * @param u the current control
	 * @param previous previous control
	 * @return two possible durations of switching time
	 */
	public static TransformationTime[] completeSegment2Reverse(Transformation TLR, Control u, Control previous) {
		Transformation reflectedTLR = TLR.reflect();
		Control reverseU = u.reverse();
		Control reversePrevious = previous.reverse();
		double[] duration = timeToSwitch2(reflectedTLR, reverseU, reversePrevious, false);
		Transformation T1 = null, T2 = null;
		if (Double.isFinite(duration[0])) {
			T1 = reflectedTLR.move(reverseU, duration[0]).reflect();
		}
		if (Double.isFinite(duration[1])) {
			T2 = reflectedTLR.move(reverseU, duration[1]).reflect();
		}
		return new TransformationTime[] {new TransformationTime(T1, duration[0]),
										 new TransformationTime(T2, duration[1])};
	}
}
