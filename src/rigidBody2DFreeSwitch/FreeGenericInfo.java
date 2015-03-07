package rigidBody2DFreeSwitch;
import optimalControl.Control;
import optimalControl.ControlLine;
import optimalControl.ControlStructure;
import optimalControl.DistanceTime;
import optimalControl.Trajectory;
import optimalControl.Transformation;

/**
 * Information to construct a generic trajectory
 * @author yu-hanlyu
 *
 */
public class FreeGenericInfo implements GenericInfo{
	public static final GenericInfo INFINITY = new Builder(DistanceTime.INFINITY, -1).build();
	private Control us;
	private Control uf;
	private double ts;
	private double tf;
	private ControlStructure structure;
	private long cycles;
	private int phaseLength;
	private Transformation TLR;
	private DistanceTime distanceTime;
	private double H;
	private ControlLine controlLine;
	
	/**
	 * Builder pattern constructor
	 * @param builder
	 */
	public FreeGenericInfo(Builder builder) {
		distanceTime = builder.distanceTime;
		H = builder.H;
		us = builder.us;
		uf = builder.uf;
		ts = builder.ts;
		tf = builder.tf;
		cycles = builder.cycles;
		phaseLength = builder.phaseLength;
		structure = builder.structure;
		TLR = builder.TLR;
		controlLine = builder.controlLine;
	}
	
	/**
	 * Return the control structure
	 * @return the control structure
	 */
	@Override
	public ControlStructure getStructure() {
		return structure;
	}

	/**
	 * Return the distance to the goal and time for the trajectory
	 * @return distance and time
	 */
	@Override
	public DistanceTime getDistanceTime() {
		return distanceTime;
	}

	/**
	 * Construct a trajectory
	 * @return a trajectory
	 */
	@Override
	public Trajectory getTrajectory() {
		Trajectory trajectory = new Trajectory();
		Trajectory period = structure.buildTrajectory(H);
		trajectory.addControl(us, ts);
	    for (int i = 0; i < cycles; ++i) {
	        for (int j = 0; j < period.size(); ++j)
	        	trajectory.addControl(period.getControl(j), period.getDuration(j));
	    }

	    for (int i = 0; i < phaseLength - 1; ++i)
	    	trajectory.addControl(period.getControl(i), period.getDuration(i));
	    trajectory.addControl(uf, tf);
	    
	    return trajectory;
	}
	
	/**
	 * 
	 * @return
	 */
	@Override
	public ControlLine getControlLine() {
		return controlLine;
	}
	
	@Override
	public String toString() {
		if (this == INFINITY)
			return "INFINITY";
		return us.toString() + " " + uf.toString() + " " + H + " " + distanceTime.getDistance() + " " + distanceTime.getTime();
	}
	
	/**
	 * Builder pattern
	 * @author yu-hanlyu
	 *
	 */
	public static class Builder {
		private Control us;
		private Control uf;
		private double ts;
		private double tf;
		private ControlStructure structure;
		private long cycles;
		private int phaseLength;
		private Transformation TLR;
		private DistanceTime distanceTime;
		private double H;
		private ControlLine controlLine;
		
		/**
		 * Constructor
		 * @param distanceTime
		 * @param H
		 */
		public Builder(DistanceTime distanceTime, double H) {
			this.distanceTime = distanceTime;
			this.H = H;
		}
		
		/**
		 * Set the first control and the last control
		 * @param us the first control
		 * @param uf the last control
		 * @return a builder
		 */
		public Builder usAndUf(Control us, Control uf) {
			this.us = us;
			this.uf = uf;
			return this;
		}
		
		/**
		 * Set the first duration and the last duration
		 * @param ts the first duration
		 * @param tf the last duration
		 * @return a builder
		 */
		public Builder tsAndTf(double ts, double tf) {
			this.ts = ts;
			this.tf = tf;
			return this;
		}
		
		/**
		 * Set the control structure
		 * @param structure the control structure
		 * @return a builder
		 */
		public Builder structure(ControlStructure structure) {
			this.structure = structure;
			return this;
		}
		
		/**
		 * Set the number of periods and phase length
		 * @param cycles the number of periods
		 * @param phaseLength phase length
		 * @return a builder
		 */
		public Builder cyclesAndPhase(long cycles, int phaseLength) {
			this.cycles = cycles;
			this.phaseLength = phaseLength;
			return this;
		}
		
		/**
		 * Set the configuration determining the structure 
		 * @param TLR a configuration in the control line frame
		 * @return a builder
		 */
		public Builder TLR(Transformation TLR) {
			this.TLR = TLR;
			return this;
		}
		
		/**
		 * Set the control line associated with the generic trajectory
		 * @param controlLine a control line
		 * @return a builder
		 */
		public Builder controlLine(ControlLine controlLine) {
			this.controlLine = controlLine;
			return this;
		}
		
		/**
		 * Build the generic trajectory info
		 * @return
		 */
		public FreeGenericInfo build() {
			return new FreeGenericInfo(this);
		}
	}
}