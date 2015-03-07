package rigidBody2DFreeSwitch;

import java.util.List;

import optimalControl.DistanceTimeL;

public interface DistanceFunctor {
	public DistanceTimeL getL(double H, long multiplier);
	
	public long getCycles();
	
	public boolean hasStructure();
	
	public boolean hasMultipleSolution();
	
	public GenericInfo computeTrajectory(double H);
	
	public List<GenericInfo> computeAllTrajectories(double H, double distError);
}
