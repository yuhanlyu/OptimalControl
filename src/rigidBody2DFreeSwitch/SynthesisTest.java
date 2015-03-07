package rigidBody2DFreeSwitch;
import optimalControl.Interval;

import org.junit.Test;

import robotModel.DiffDrive;
import robotModel.DubinsCar;
import robotModel.OmniDrive;
import robotModel.RSCar;
import robotModel.Robot;


public class SynthesisTest {
	private static final Interval rangeX = new Interval(-2, 2);
	private static final Interval rangeY = new Interval(-2, 2);
	private static final double RESOLUTION = 0.1;
	private static final double STEP = Math.PI / 15;

	public static void test() {
		System.out.println("Dubins Car");
		testRobot(new DubinsCar());
		System.out.println("Reed-Shepp Car");
		 testRobot(new RSCar()); 
		System.out.println("Omnidirectional Vehicle");
		testRobot(new OmniDrive());
		System.out.println("Differential Drive");
		//testRobot(new DiffDrive());
	}
	
	public static void testRobot(Robot robot) {
		for (double theta = 0; theta <= Math.PI * 2; theta += STEP) {
			System.out.println("Theta is: " + theta);
			testSynthesis(robot, theta);
		}
	}
	
	public static void testSynthesis(Robot robot, double theta) {
		Synthesis synthesis = new Synthesis.SynthesisBuilder(robot).setX(rangeX.getBegin(), rangeX.getEnd())
                                           .setY(rangeY.getBegin(), rangeY.getEnd())
                                           .setTheta(theta)
                                           .setResolution(RESOLUTION)
                                           .build();
	}
	
	public static void main(String[] argv) {
		test();
	}
}
