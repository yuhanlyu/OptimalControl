package optimalControl;
import static org.junit.Assert.*;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.junit.Test;

import rigidBody2DFreeSwitch.DistanceMinimization;
import robotModel.DiffDrive;
import robotModel.DubinsCar;
import robotModel.OmniDrive;
import robotModel.RSCar;
import robotModel.Robot;


public class ControlLineTest {

	private static final double MAX = 10.0;
	private static final int TEST_CASES = 10000;
	private static final double DELTA = 0.000001;
	private static final double STEP = 0.001;
	
	@Test
	public void test() {
		Configuration qf = new Configuration(0, 0, 0);
		test1();
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
		ControlSet U = robot.getControlSet();
		Transformation Ts = new Transformation(qs);
		Transformation Tf = new Transformation(qf);
		for (int i = 0; i < U.size(); ++i) {
			for (int j = 0; j < U.size(); ++j) {
				Control us = U.getControl(i);
				Control uf = U.getControl(j);
				if (us.getOmega() == 0.0 && uf.getOmega() == 0.0)
					continue;
				boolean isPositive = Math.random() > 0.5;
				ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, isPositive);
				testFactory(U, Ts, us, Tf, uf, factory);
				testSwitchPointOnLine(U, Ts, us, factory);
				testRange(U, Ts, us, Tf, uf, factory);
				testL(U, Ts, us, Tf, uf, factory);
				//testL(U, Ts, us, Tf, uf, factory, false);
			}
		}	
	}
	
	public static void testL(ControlSet U, Transformation Ts, Control us, Transformation Tf, Control uf, ControlLineFactory factory) {
		Interval range = factory.getRange();
		int begin, end;
	    // Find the first critical value larger than lb
	    for (begin = 0; begin < U.getCriticalSize(); ++begin) {
	        if (U.getCriticalValue(begin) > range.getBegin())
	            break;
	    }
	    // Find the last critical value smaller than ub
	    for (end = U.getCriticalSize() - 1; end >= 0; --end) {
	        if (U.getCriticalValue(end) < range.getEnd())
	            break;
	    }
	    
	    ArrayList<DistanceMinimization> result = new ArrayList<>();
	    // Generate all ranges of H
	    if (begin == U.getCriticalSize() || end == -1 || begin == end + 1) {
	    	testL(U, Ts, us, Tf, uf, factory, range.getBegin(), range.getEnd());
	        return;
	    }
	    testL(U, Ts, us, Tf, uf, factory, range.getBegin(), U.getCriticalValue(begin));
	    for (int i = begin; i < end; ++i)
	    	testL(U, Ts, us, Tf, uf, factory, U.getCriticalValue(i), U.getCriticalValue(i+1));
	    testL(U, Ts, us, Tf, uf, factory,U.getCriticalValue(end), range.getEnd());
	}
	
	public static void testL(ControlSet U, Transformation Ts, Control us, Transformation Tf, Control uf, ControlLineFactory factory, double lb, double ub) {
		for (double H = lb + DELTA + STEP; H < ub - DELTA; H += STEP) {
			double preH = H - STEP;
		    ControlLine controlLine = factory.getControlLine(H);
		    ControlLine preControlLine = factory.getControlLine(preH);
		    /*
		    ControlLineFactory.ControlLineL controlLineL = factory.getControlLineL(H);
		    double dkx = Math.abs(controlLine.getKx() - preControlLine.getKx());
		    double dky = Math.abs(controlLine.getKy() - preControlLine.getKy());
		    double dktheta = Math.abs(controlLine.getKtheta() - preControlLine.getKtheta());
		    if (dkx > controlLineL.getKxL() * STEP && !Utility.absEqual(controlLineL.getKxL()* STEP, dkx)) {
		    	System.out.println("Error " + dkx + " " + controlLineL.getKxL() * STEP);
		    }
		    if (dky > controlLineL.getKyL() * STEP && !Utility.absEqual(controlLineL.getKyL() * STEP, dky)) {
		    	System.out.println("Error " + dky + " " + controlLineL.getKyL() * STEP);
		    }
		    if (dktheta > controlLineL.getKthetaL() * STEP && !Utility.absEqual(controlLineL.getKthetaL() * STEP, dktheta)) {
		    	System.out.println("Error " + dktheta + " " + controlLineL.getKthetaL() * STEP);
		    }
		    assertFalse("Dkx ", dkx > controlLineL.getKxL() * STEP && !Utility.absEqual(controlLineL.getKxL() * STEP, dkx));
		    assertFalse("Dky ", dky > controlLineL.getKyL() * STEP && !Utility.absEqual(controlLineL.getKyL() * STEP, dky));
		    assertFalse("Dktheta ", dktheta > controlLineL.getKthetaL() * STEP && !Utility.absEqual(controlLineL.getKthetaL() * STEP, dktheta));
		    
		    Transformation TLW = new Transformation(controlLine);
		    Transformation preTLW = new Transformation(preControlLine);
		    Transformation TLR = TLW.transform(Ts);
		    Transformation preTLR = preTLW.transform(Ts);
		    double distX = Math.abs(TLR.getX() - preTLR.getX());
		    double distY = Math.abs(TLR.getY() - preTLR.getY());
		    double distCos = Math.abs(TLR.getCos() - preTLR.getCos());
		    double distSin = Math.sin(TLR.getSin() - preTLR.getSin());
		    ControlLineFactory.PositionL mappingL = factory.getMappingL(H, Ts);
		    if (distX > mappingL.getXL() * STEP && !Utility.absEqual(mappingL.getXL() * STEP, distX)) {
		    	System.out.println("Error " + distX + " " + mappingL.getXL() * STEP);
		    }
		    if (distY > mappingL.getYL() * STEP && !Utility.absEqual(mappingL.getYL() * STEP, distY)) {
		    	System.out.println("Error " + distY + " " + mappingL.getYL() * STEP);
		    }
		    assertFalse("mapping x", distX > mappingL.getXL() * STEP && !Utility.absEqual(mappingL.getXL() * STEP, distX));
		    assertFalse("mapping y", distY > mappingL.getYL() * STEP && !Utility.absEqual(mappingL.getYL() * STEP, distY));
		    assertFalse("mapping cos", distCos > mappingL.getCosL() * STEP && !Utility.absEqual(mappingL.getCosL() * STEP, distCos));
		    assertFalse("mapping sin", distSin > mappingL.getSinL() * STEP && !Utility.absEqual(mappingL.getSinL() * STEP, distSin));
		    */
		    Transformation TLR = new Transformation(controlLine).transform(Ts);
		    TransformationTime tt = U.completeSegment(TLR, us);
		    double time = tt.getTime();
		    double x = tt.getTransformation().getX();
		    
		    Transformation preTLR = new Transformation(preControlLine).transform(Ts);
		    TransformationTime preTt = U.completeSegment(preTLR, us);
		    double preTime = preTt.getTime();
		    double preX = preTt.getTransformation().getX();
		    
		    double distTime = Math.abs(time - preTime);
		    double distD    = Math.abs(x - preX);
		    
		    Control next = U.nextSustainableControl(TLR, us);
		    ControlLineFactory.LFunctor functor = factory.getLFunctor(Ts, us, next);
		    DistanceTimeL dt = functor.getL(H);
		    
		    Transformation RCW = new Transformation(us.rotationCenter(Ts));
		    Point2D RC = us.rotationCenter(TLR);
		    Point2D preRC = us.rotationCenter(preTLR);
		    double distSP = Math.abs(RC.getX() - preRC.getX());
		    ControlLineFactory.PositionL rcL = factory.getMappingL(H, RCW);
		    if (distSP > STEP * rcL.getXL()) {
		    	System.out.println("Error: " + distSP + " " + STEP * rcL.getXL());
		    }
		    
		    if (distTime > dt.getTimeL() * STEP) {
		    	System.out.println("Error distTime: " + distTime + " " + dt.getTimeL() * STEP);
		    }
		    if (distD > dt.getDistanceL() * STEP) {
		    	System.out.println("Error distL: " + distD + " " +  dt.getDistanceL() * STEP);
		    	System.out.println("H: " + H + " " + lb + " " + ub);
		    	System.out.println(new Configuration(Ts) + " us: " + us + " uf: " + uf + " " + H);
		    	System.out.println(time + " " + preTime);
		    }
		    assertTrue("dist L", distD <= dt.getDistanceL() * STEP);
		    assertTrue("dist L", distTime <= dt.getTimeL() * STEP);
		}
	}
	
	
	/**
	 * 
	 * @param U
	 * @param Ts
	 * @param us
	 * @param Tf
	 * @param uf
	 * @param controlLine
	 * @param range
	 * @param H
	 */
	public static void testRangeHelper(ControlSet U, Transformation Ts, Control us, Transformation Tf, Control uf, ControlLine controlLine, Interval range, double H) {
		 if (!controlLine.isValid())
		    return;
		 if (H > range.getEnd())
			 return;
		 if (Math.abs(range.getBegin() - H) < DELTA)
			 return;
		 if (Math.abs(range.getEnd() - H) < DELTA)
			 return;
		 Transformation TLW = new Transformation(controlLine);
		 Transformation TsL = TLW.transform(Ts);
		 Transformation TfL = TLW.transform(Tf);
		 if (range.contains(H)) {
			 assertTrue("Test range", U.isMaximizing(TsL, us));
			 assertTrue("Test range", U.isMaximizing(TfL, uf));
		 } else {
			 if (U.isMaximizing(TsL, us) && U.isMaximizing(TfL, uf)) {
				 System.out.println("qs: " + new Configuration(Ts) + " us: " + us + " qf: " + new Configuration(Tf) + " uf: " + uf + " control line: " + controlLine);
				 System.out.println("H: " + H + " upper bound: " + range.getEnd() + " " + range.getBegin());
			 }
			 assertTrue("Test range", !U.isMaximizing(TsL, us) || !U.isMaximizing(TfL, uf));
		 }
	}
	
	/**
	 * 
	 * @param U
	 * @param Ts
	 * @param us
	 * @param Tf
	 * @param uf
	 * @param factory
	 */
	public static void testRange(ControlSet U, Transformation Ts, Control us, Transformation Tf, Control uf, ControlLineFactory factory) {
		Interval range = factory.getRange();
		for (double H = 0.01; H < U.getUpperBound() && H < factory.getUpperBoundOfH(); H += 0.01) {
		    ControlLine controlLine = factory.getControlLine(H);
		    testRangeHelper(U, Ts, us, Tf, uf, controlLine, range, H);
		}
	}
	
	/**
	 * 
	 * @param U
	 * @param Ts
	 * @param us
	 * @param factory
	 */
	public static void testSwitchPointOnLine(ControlSet U, Transformation Ts, Control us, ControlLineFactory factory) {
		for (int k = 0; k < U.size(); ++k) {
			Control u = U.getControl(k);
			double H = factory.switchPointOnLine(Ts, us, u);
			ControlLine controlLine = factory.getControlLine(H);
			if (!controlLine.isValid())
				continue;
			Transformation TLW = new Transformation(controlLine);
			
			double y = TLW.transform(Ts.transform(u.switchPoint(us))).getY();
			assertTrue("Test case " + ", y:", Utility.isZero(y));
		}
	}
	
	/**
	 * 
	 * @param Ts
	 * @param us
	 * @param Tf
	 * @param uf
	 * @param factory
	 */
	public static void testFactory(ControlSet U, Transformation Ts, Control us, Transformation Tf, Control uf, ControlLineFactory factory) {
		for (double H = 0.01; H < U.getUpperBound(); H += 0.01) {
		    boolean positive = Math.random() > 0.5;
		    ControlLine controlLine = factory.getControlLine(H);
		    if (!controlLine.isValid())
		    	continue;
		    Transformation TLW = new Transformation(controlLine);
		    double H1 = ControlSet.Hamiltonian(TLW.transform(Ts), us);
		    double H2 = ControlSet.Hamiltonian(TLW.transform(Tf), uf);
		    assertTrue("Test case " + ", H1:", Utility.absEqual(H1, H));
		    assertTrue("Test case " + ", H2:", Utility.absEqual(H2, H));
		}
	}
	
	/**
	 * Test
	 */
	public static void test1() {
		Configuration qs = new Configuration(-1.8633989092588255, 4.196746459882739, -1.7543161987095035);
		Configuration qf = new Configuration(0, 0, 0);
		Transformation Ts = new Transformation(qs);
		Transformation Tf = new Transformation(qf);
		Control us = new Control(1.1547005383792515, -0.6666666666666666, -0.3333333333333333);
		Control uf = new Control(0.0, -1.3333333333333333, 0.3333333333333333);
		ControlSet U = (new OmniDrive()).getControlSet();
	    ControlLineFactory factory = new ControlLineFactory(U, Ts, us, Tf, uf, true);
	    Interval range = factory.getRange();
	    System.out.println(range);
		double H = 1.2092233025419197;
		double preH = H - STEP;
	    ControlLine controlLine = factory.getControlLine(H);
	    ControlLine preControlLine = factory.getControlLine(preH);
	    Transformation TLR = new Transformation(controlLine).transform(Ts);
	    TransformationTime tt = U.completeSegment(TLR, us);
	    double time = tt.getTime();
	    double x = tt.getTransformation().getX();
	    
	    Transformation preTLR = new Transformation(preControlLine).transform(Ts);
	    TransformationTime preTt = U.completeSegment(preTLR, us);
	    double preTime = preTt.getTime();
	    double preX = preTt.getTransformation().getX();
	    
	    double distTime = Math.abs(time - preTime);
	    double distD    = Math.abs(x - preX);
	    
	    Control next = U.nextSustainableControl(TLR, us);
	    Control preNext = U.nextSustainableControl(preTLR, us);
	    System.out.println(next.equals(preNext));
	    ControlLineFactory.LFunctor functor = factory.getLFunctor(Ts, us, next);
	    DistanceTimeL dt = functor.getL(H);
	    System.out.println("SP: " + TLR.transform(us.switchPoint(next)));
	    System.out.println("RC: " + TLR.transform(new Homogeneous(us)));
	    
	    //if (distTime > dt.getTimeL() * STEP) {
	    //	System.out.println("Error distTime: " + distTime + " " + dt.getTimeL() * STEP);
	    //}
	    System.out.println("Time: " + time + " " + preTime);
	    if (distD > dt.getDistanceL() * STEP) {
	    	System.out.println("Error distL: " + distD + " " +  dt.getDistanceL() * STEP);
	    	System.out.println(new Configuration(Ts) + " us: " + us + " uf: " + uf + " " + H);
	    	System.out.println(time + " " + preTime);
	    }
	    assertTrue("dist L", distD <= dt.getDistanceL() * STEP);
	    //assertTrue("dist L", distTime <= dt.getTimeL() * STEP);
	}
}
