package optimalControl;
import static org.junit.Assert.*;

import org.junit.Test;

import robotModel.DiffDrive;
import robotModel.DubinsCar;
import robotModel.OmniDrive;
import robotModel.RSCar;
import robotModel.Robot;


public class ControlSetTest {
	private static final double MAX = 10.0;
	private static final int TEST_CASES = 1000;
	
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
				for (double H = 0.01; H < 2; H += 0.01) {
					boolean isPositive = Math.random() > 0.5;
					ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, isPositive);
				    
				    ControlLine controlLine = factory.getControlLine(H);
				    if (!controlLine.isValid())
				    	continue;
				    Transformation TLW = new Transformation(controlLine);
				    Transformation TLR = TLW.transform(Ts);
				    for (int k = 0; k < U.size(); ++k) {
				    	Control u = U.getControl(k);
				    	if (!u.equals(us)) {
				    		double t = ControlSet.timeToSwitch(TLR, us, u, false);
				    		if (!Double.isFinite(t))
				    			continue;
				    		Transformation Tu = new Transformation(us, t);
				    		Transformation T = TLR.transform(Tu);
				    		double H1 = ControlSet.Hamiltonian(T, us);
				    		double H2 = ControlSet.Hamiltonian(T, u);
				    		//System.out.println(H + " " + H1 + " " + H2);
				    		assertTrue("Test case " + ", H1:", Utility.absEqual(H1, H2));
				    	}
				    }
				}
			}
		}
	}

	public static void test1() {
		double t = ControlSet.timeToNullY(new Homogeneous(0.0, -0.4, -2.0), new Homogeneous(-1.0, 0.2, 1.0), false);
	    System.out.println(t + " output should be: 3.34295");
	    double t2 = ControlSet.timeToSwitch(new Transformation(new Configuration(0, 0.2, Math.PI/3.0)), new Control(1, 0, 1), new Control(1, 0, -1), false);
	    System.out.println(t2 + " output should be: 4.44059");
	    double t3 = ControlSet.timeToSwitch(new Transformation(new Configuration(0, -2, Math.PI/2.0)), new Control(1, 0, 0), new Control(0, 0, 1), false);
	    System.out.println(t3 + " output should be: 2");
	    double t4 = ControlSet.timeToSwitch(new Transformation(new Configuration(0, 0, 0.01)), new Control(1.1547005383792512, -0.66666666666666685, -0.33333333333333315), new Control(0, 0, -1), false);
	    System.out.println(t4 + " output should be: 1.31299");
	    ControlSet U = (new OmniDrive()).getControlSet();
	    ControlTime ct = U.getGenericSegment(new Transformation(new Configuration(0, 0, 0.01)));
	    System.out.println(ct.getControl() + " " + ct.getTime() + " output should be: (1.1547, -0.666667, -0.333333) 1.31299");
	    Control u1 = new Control(1.1547005383792512, -0.66666666666666685, -0.33333333333333315);
	    Control u2 = new Control(0, 0, -1);
	    Transformation T0 = new Transformation(new Configuration(0, 0, 0.01));
	    double t5 = ControlSet.timeToSwitch(T0, u1, u2, false);
	    Transformation T1 = T0.move(u1, t5);
	    ct = U.getGenericSegment(T1);
	    System.out.println(ct.getControl() + " " + ct.getTime() + " output should be: (0, 0, -1) 0.191867\n");
	    
	    T0 = new Transformation(new Configuration(-3.0, 0, 0.1));
	    ct = U.getGenericSegment(T0);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    T1 = T0.move(ct.getControl(), ct.getTime());
	    ct = U.getGenericSegment(T1);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    Transformation T2 = T1.move(ct.getControl(), ct.getTime());
	    ct = U.getGenericSegment(T2);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    Transformation T3 = T2.move(ct.getControl(), ct.getTime());
	    ct = U.getGenericSegment(T3);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    System.out.println("output should be: (1.1547, -0.666667, -0.333333) 1.48194\n(0, 0, -1) 0.259235\n(0, 1.33333, -0.333333) 5.50548\n(0, 0, -1) 0.259235\n");
	    
	    /*
	    T0 = new Transformation(new Configuration(-3.0, -1.5, 0.1));
	    Trajectory traj = U.getGenericSegment(T0, 6);
	    System.out.println(traj);
	    System.out.println("output's time should be 0.348999 1.01453 3.23959 1.01453 3.23959 1.01453\n");
	    T0 = new Transformation(new Configuration(-3.0, 0.7, -Math.PI/2));
	    traj = U.getGenericSegment(T0, 3);
	    System.out.println(traj);	    
	    System.out.println("output's time should be: 0.643696 0.467399 0.117531\n");
	    */
	    Trajectory traj;
	    T0 = new Transformation(new Configuration(-3.0, 0.7, -Math.PI/2));
	    traj = U.getPeriod(T0);
	    System.out.println(traj);	    
	    System.out.println("output's time should be: 0.643696 0.467399 0.117531 0.467399\n");
	    
	    T0 = new Transformation(new Configuration(-3.0, -1.5, 0.1));
	    traj = U.getPeriod(T0);
	    System.out.println(traj);	    
	    System.out.println("output's time should be: 0.348999 1.01453 3.23959 1.01453 3.23959 1.01453\n");
	    
	    U = (new DiffDrive()).getControlSet();
	    T0 = new Transformation(new Configuration(-2.0, 0, Math.PI / 3));
	    ct = U.getGenericSegment(T0);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    T1 = T0.move(ct.getControl(), ct.getTime());
	    ct = U.getGenericSegment(T1);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    T2 = T1.move(ct.getControl(), ct.getTime());
	    ct = U.getGenericSegment(T2);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    T3 = T2.move(ct.getControl(), ct.getTime());
	    ct = U.getGenericSegment(T3);
	    System.out.println(ct.getControl() + " " + ct.getTime());
	    System.out.println("output's time should be: 0.57735 1.0472 1.1547 1.0472\n");
	    
	    /*
	    T0 = new Transformation(new Configuration(-2.0, 0, Math.PI / 3));
	    traj = U.getGenericSegment(T0, 3);
	    System.out.println(traj);
	    System.out.println("output's time should be: 0.57735 1.0472 1.1547\n");
	    */
	}
}
