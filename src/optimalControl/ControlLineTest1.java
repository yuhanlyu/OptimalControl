package optimalControl;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.Rectangle2D;

import javax.swing.JFrame;
import javax.swing.Timer;

import robotModel.Robot;
import ui.TrajectoryDrawing;
import ui.TrajectoryViewer;



public class ControlLineTest1 {
	private static double STEP = 0.01;
	private static TrajectoryViewer viewer = new TrajectoryViewer();
	private static double H = 0.0;

	public static void main(String[] args) {
		javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
			public void run() {
                createAndShowGUI();
            }
        });
	}
	
	/**
	 * the driver to create GUI
	 */
	private static void createAndShowGUI() {
        //Create and set up the window.
        JFrame frame = new JFrame("Viewer");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        viewer.setSize(400, 500);
        frame.getContentPane().add(viewer);
        //TrajectoryParser parse = new TrajectoryParser("data/trajectory");
        frame.setSize(400, 500);
        frame.setVisible(true);
        testControlLine();
    }
	
	public static void testControlLine() {
		Robot robot = new Robot(null, new Rectangle2D.Double(-0.6, -0.3, 1.2, 0.6));
		Configuration initial = new Configuration(-3, -3, Math.PI/2.0);
	    Transformation qs = new Transformation(new Configuration(-3, -3, Math.PI/2.0));
	    Control us = new Control(1, 0, 1);
	    Transformation qf = new Transformation(new Configuration(0, 0, 0));
	    Control uf = new Control(1, 0, 0);
	    ControlLineFactory factory = new ControlLineFactory(robot.getControlSet(), qs, us, qf, uf, true);
	    Trajectory trajectory = new Trajectory();
	    ControlLine L = factory.getControlLine(H);
		TrajectoryDrawing draw = new TrajectoryDrawing(initial, robot, trajectory, L);
		viewer.setDrawing(draw);
		System.out.println(H);
		new Timer(100, new ActionListener(){
		    @Override
			public void actionPerformed(ActionEvent evt){
		    	ControlLine L = factory.getControlLine(H);
		    	TrajectoryDrawing draw = new TrajectoryDrawing(initial, robot, trajectory, L);
		    	viewer.setDrawing(draw);
		    	H += STEP;
		    	System.out.println(L.toString());
		    	if (H <= 1.0)
		    		viewer.repaint();
		    }
		}).start();
	}

}
