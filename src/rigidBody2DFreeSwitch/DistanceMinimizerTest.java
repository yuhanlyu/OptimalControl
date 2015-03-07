package rigidBody2DFreeSwitch;
import optimalControl.Configuration;
import optimalControl.ControlSet;
import optimalControl.Transformation;

import org.junit.Test;

import robotModel.OmniDrive;
import robotModel.Robot;

public class DistanceMinimizerTest {
	private static final int TEST_CASES = 100;
	private static final double MAX = 10.0;

	@Test
	public void test() {
		testUniformSampleMinimizer();
	}

	public static void testUniformSampleMinimizer() {
		DistanceMinimizer minimizer = new UniformSampleMinimizer();
		testMinimizer(minimizer);
	}
	
	public static void testMinimizer(DistanceMinimizer minimizer) {
		testRobot(new OmniDrive(), minimizer);
	}
	
	public static void testRobot(Robot robot, DistanceMinimizer minimizer) {
		ControlSet U = robot.getControlSet();
		for (int i = 0; i < TEST_CASES; ++i) {
			Configuration qs = new Configuration((Math.random() - 0.5) * MAX, 
                                                 (Math.random() - 0.5) * MAX, 
                                                 (Math.random() * 2 * Math.PI));
			testFindingPath(U, qs, minimizer);
		}
	}
	
	public static void testFindingPath(ControlSet U, Configuration qs, DistanceMinimizer minimizer) {
		Transformation Ts = new Transformation(qs);
		GenericSolver solver = new GenericSolver(U, Ts, minimizer);
		TrajectoryInfo solution = solver.solve();
		System.out.println(new Configuration(Ts.move(solution.getTrajectory())));
	}
}
