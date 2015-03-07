package optimalControl;
import static org.junit.Assert.*;

import org.junit.Test;

import robotModel.DiffDrive;
import robotModel.DubinsCar;
import robotModel.OmniDrive;
import robotModel.RSCar;
import robotModel.Robot;


public class DTFunctorTest {
	private static final int TEST_CASES = 1000;
	private static final double MAX = 10.0;
	private static final double DELTA = 0.00001;
	private static final double STEP = 0.001;

	@Test
	public void test() {
		Configuration qf = new Configuration(0, 0, 0);
		//test1();
		for (int i = 0; i < TEST_CASES; ++i) {
			// Random generate configuration and controls
			Configuration qs = new Configuration((Math.random() - 0.5) * MAX, 
					                             (Math.random() - 0.5) * MAX, 
				                                 (Math.random() * 2 * Math.PI));
			testRobot(new DubinsCar(), qs, qf); 
			testRobot(new RSCar(), qs, qf); 
			testRobot(new DiffDrive(), qs, qf); 
			testRobot(new OmniDrive(), qs, qf); 
		}
	}

	public static void testRobot(Robot robot, Configuration qs, Configuration qf) {
		Transformation Tf = new Transformation(qf);
		Transformation Ts = new Transformation(qs);
		ControlSet U = robot.getControlSet();
		for (int i = 0; i < U.size(); ++i) {
			for (int j = 0; j < U.size(); ++j) {
				Control us = U.getControl(i);
				Control uf = U.getControl(j);
				if (us.getOmega() == 0.0 && uf.getOmega() == 0.0)
					continue;
				boolean isPositive = Math.random() > 0.5;
				ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, isPositive);
				Interval range = factory.getRange();
				if (range.isEmpty())
					continue;
				for (double H = range.getBegin() + DELTA; H < range.getEnd(); H += STEP) {
				    ControlLine controlLine = factory.getControlLine(H);
				    assertTrue("", controlLine.isValid());
				    Transformation TLW = new Transformation(factory.getControlLine(H));
				    Transformation TsL = TLW.transform(Ts);
					Transformation TfL = TLW.transform(Tf);
					TransformationTime spair = U.completeSegment(TsL, us);
					TransformationTime fpair = U.completeSegment(TfL, uf);
					Control afterUf = U.sustainableControl(fpair.getTransformation());
					TsL = spair.getTransformation();
					
					// Find the control structure
					ControlStructure structure = new ControlStructure(U.getPeriod(TsL));
					if (structure.size() <= 1 || !structure.containSwitch(uf, afterUf)) {
						continue;
					}
					for (int k = 0; k < structure.size(); ++k) {
						TransformationTime pair = U.completeSegment(TsL, structure.getControl(k));
						double dist = pair.getTransformation().getX() - TsL.getX();
						double time = pair.getTime();
						DistanceTime dt = structure.getDistanceTime(H, k);
						if (!Utility.absEqual(time, dt.getTime()) || !Utility.absEqual(dist, dt.getDistance())) {
							System.out.println("qs: " + qs);
							System.out.println("us: " + us + " uf: " + uf + " positive: " + isPositive);
							System.out.println("H: " + H + " k: " + k);
						}
						assertTrue("Time is unequal", Utility.absEqual(time, dt.getTime()));
						assertTrue("Distance is unequal", Utility.absEqual(dist, dt.getDistance()));
						TsL = pair.getTransformation();
						
						DistanceTimeL L = structure.getL(H, k);
						double preH = H - STEP;
						if (preH < range.getBegin() + DELTA)
							continue;
						DistanceTime preDt = structure.getDistanceTime(preH, k);
						double changeInDistance = Math.abs(dt.getDistance() - preDt.getDistance());
						double changeInTime = Math.abs(dt.getTime() - preDt.getTime());
						if (changeInTime > STEP * L.getTimeL()) {
							System.out.println(changeInTime + " " + STEP * L.getTimeL() + " " + structure.getControl(k));
						}
						if (changeInDistance > STEP * L.getDistanceL()) {
							System.out.println(changeInDistance + " " + STEP * L.getDistanceL() + " " + structure.getControl(k));
						}
						assertTrue("Time's Lipschitz constant is incorrect", changeInTime <= STEP * L.getTimeL());
						assertTrue("Distance's Lipschitz constant is incorrect", changeInDistance <= STEP * L.getDistanceL());
					}
				}
			}
		}
	}
	
	public static void test1() {
		ControlSet U = new DiffDrive().getControlSet();
		Configuration qs = new Configuration(1.0203086582319798, -3.663797825375311, 5.0289708611172035);
		Configuration qf = new Configuration(0, 0, 0);
		Transformation Tf = new Transformation(qf);
		Transformation Ts = new Transformation(qs);
		Control us = new Control(0, 0, 1);
		Control uf = new Control(0, 0, 1);
		boolean isPositive = false;
		ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, isPositive);
		Interval range = factory.getRange();
		System.out.println(range);
		for (double H = range.getBegin() + DELTA; H < range.getEnd(); H += STEP) {
		    ControlLine controlLine = factory.getControlLine(H);
		    assertTrue("", controlLine.isValid());
		    Transformation TLW = new Transformation(factory.getControlLine(H));
		    Transformation TsL = TLW.transform(Ts);
			Transformation TfL = TLW.transform(Tf);
			TransformationTime spair = U.completeSegment(TsL, us);
			TransformationTime fpair = U.completeSegment(TfL, uf);
			Control afterUf = U.sustainableControl(fpair.getTransformation());
			TsL = spair.getTransformation();
			
			// Find the control structure
			ControlStructure structure = new ControlStructure(U.getPeriod(TsL));
			if (structure.size() <= 1 || !structure.containSwitch(uf, afterUf)) {
				continue;
			}
			for (int k = 0; k < structure.size(); ++k) {
				TransformationTime pair = U.completeSegment(TsL, structure.getControl(k));
				double dist = pair.getTransformation().getX() - TsL.getX();
				double time = pair.getTime();
				DistanceTime dt = structure.getDistanceTime(H, k);
				TsL = pair.getTransformation();
				if (!Utility.absEqual(time, dt.getTime()) || !Utility.absEqual(dist, dt.getDistance())) {
					System.out.println("qs: " + qs);
					System.out.println("us: " + us + " uf: " + uf + " positive: " + isPositive);
					System.out.println("H: " + H + " k: " + k);
					System.out.println(time + " " + dt.getTime());
				}
				assertTrue("Time is unequal", Utility.absEqual(time, dt.getTime()));
				assertTrue("Distance is unequal", Utility.absEqual(dist, dt.getDistance()));
			}
		}
		//double H = 0.9999985873479567;
		//int k = 2;
	}
}
