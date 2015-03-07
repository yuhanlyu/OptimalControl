package rigidBody2DCostlySwitch;

import optimalControl.ControlLine;
import optimalControl.ControlStructure;
import optimalControl.DistanceTime;
import optimalControl.Trajectory;
import rigidBody2DFreeSwitch.GenericInfo;

public class CostlyGenericInfo implements GenericInfo {
	private Trajectory trajectory;
	private ControlLine controlLine;
	private DistanceTime distanceTime;
	
	public CostlyGenericInfo(Trajectory trajectory, ControlLine controlLine, DistanceTime distanceTime) {
		this.trajectory = trajectory;
		this.controlLine = controlLine;
		this.distanceTime = distanceTime;
	}

	@Override
	public DistanceTime getDistanceTime() {
		return distanceTime;
	}

	@Override
	public ControlLine getControlLine() {
		return controlLine;
	}

	@Override
	public Trajectory getTrajectory() {
		return trajectory;
	}

	@Override
	public ControlStructure getStructure() {
		return null;
	}
}
