package rigidBody2DFreeSwitch;

import optimalControl.ControlLine;
import optimalControl.ControlStructure;
import optimalControl.DistanceTime;
import optimalControl.Trajectory;

public interface GenericInfo {
	public DistanceTime getDistanceTime();
	public ControlLine getControlLine();
	public Trajectory getTrajectory();
	public ControlStructure getStructure();
}
