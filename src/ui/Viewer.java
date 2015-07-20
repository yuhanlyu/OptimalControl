package ui;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.geom.AffineTransform;
import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;

import optimalControl.Configuration;
import optimalControl.ControlLine;
import optimalControl.Interval;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import obstacleModel.Circle;
import obstacleModel.Environment;
import rigidBody2DFreeSwitch.FreePlanner;
import rigidBody2DFreeSwitch.Synthesis;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import rigidBody2DPlannerWithObstacle.OBPlanner;
import robotModel.Bench;
import robotModel.DiffDrive;
import robotModel.DubinsCar;
import robotModel.OmniDrive;
import robotModel.RSCar;
import robotModel.Robot;


public class Viewer extends JFrame {
	public static final int SCALE = 50;
	public static final AffineTransform scale = AffineTransform.getScaleInstance(Viewer.SCALE, Viewer.SCALE);
	public static final int VIEWER_WIDTH = 800;
	public static final int VIEWER_HEIGHT = 600;
	private static final int SELECTION_WIDTH = 230;
	private static final int BOUNDARY_HEIGHT = 50;
	private static final HashMap<String, Robot> robotMap = new LinkedHashMap<>();
	static {
		robotMap.put("Dubins Car", new DubinsCar());
		robotMap.put("Reeds-Shepp Car", new RSCar());
		robotMap.put("Differential drive", new DiffDrive());
		robotMap.put("Omnidirectional vehicle", new OmniDrive());
		robotMap.put("Bench", new Bench());
	}
	private static final HashMap<String, Integer> typeMap = new LinkedHashMap<>();
	static {
		typeMap.put("Feasible", Synthesis.FEASIBLE_SOLUTION);
		typeMap.put("TGT", Synthesis.TGT_SOLUTION);
		typeMap.put("Whirl", Synthesis.WHIRL_SOLUTION);
		typeMap.put("Singular", Synthesis.SINGULAR_SOLUTION);
		typeMap.put("Generic", Synthesis.GENERIC_SOLUTION);
	}
	private static final String[] ROBOT_NAMES = robotMap.keySet().toArray(new String[0]);
	private static final String[] TRAJECTORY_TYPES = {"Feasible", "TGT", "Whirl", "Singular", "Generic"};
	private Robot robot = robotMap.get(ROBOT_NAMES[0]);
	private Configuration qs = new Configuration(-3, -3, Math.PI/3.0);
	private Configuration qf = new Configuration(3, 3, Math.PI/2.0);
	private Interval rangeX = new Interval(-2, 2);
	private Interval rangeY = new Interval(-2, 2);
	private double synthesisTheta = 0;
	private double synthesisResolution = 0.1;
	
	private Environment env = new Environment();
	{
		//env.addObstacle(new Circle(new Point2D.Double(3.0, 1.0), 1.0));
		//env.addObstacle(new Circle(new Point2D.Double(-1.0, -1.0), 1.0));
		//env.addObstacle(new Circle(new Point2D.Double(0, 2), 1.0));
	}
	
	private SelectionPane selection = new SelectionPane();
	private FreePlanner obFreePlanner = new FreePlanner();
	//private OBPlanner obPlanner = new OBPlanner(robot.getControlSet(), env);
	private ObstacleViewer obstacleViewer = new ObstacleViewer(env);
	{
		//ObstacleDrawing drawing = new ObstacleDrawing(env, obPlanner.getGraph());
		Transformation Ts = new Transformation(qs);
		Transformation Tf = new Transformation(qf);
		//Trajectory sol = obPlanner.solve(Ts, Tf);
		//if (sol != null)
		//	drawing.addTrajectory(Ts, sol);
		//obstacleViewer.setDrawing(drawing);
	}
	private TrajectoryViewer trajectoryViewer = new TrajectoryViewer();
	private SynthesisViewer synthesisViewer = new SynthesisViewer();
	private AnimateViewer animateViewer = new AnimateViewer();
	{
		obstacleViewer.setSize(VIEWER_WIDTH, VIEWER_HEIGHT);
		trajectoryViewer.setSize(VIEWER_WIDTH, VIEWER_HEIGHT);
		synthesisViewer.setSize(VIEWER_WIDTH, VIEWER_HEIGHT);
		animateViewer.setSize(SELECTION_WIDTH, VIEWER_HEIGHT);
	}
	private JTabbedPane tabbedPane = new JTabbedPane();
	{
		tabbedPane.addTab("Simulator", obstacleViewer);
        tabbedPane.addTab("Trajectory", trajectoryViewer);
        tabbedPane.addTab("Synthesis", synthesisViewer);
        tabbedPane.addTab("Animate", animateViewer);
	}
	
	public Viewer(String title) {
		super(title);
        JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, tabbedPane, selection);
        selection.setSize(VIEWER_WIDTH, VIEWER_HEIGHT);
        splitPane.setDividerLocation(VIEWER_WIDTH);
        splitPane.setDividerSize(0);
        
        getContentPane().add(splitPane);
        TrajectoryParser parse = new TrajectoryParser("data/trajectory");
        
        Robot robot = parse.getRobot();
        Configuration initial = parse.getInitial();
        ControlLine L = parse.getControlLine();
        Trajectory trajectory = parse.getTrajectory();
        trajectoryViewer.setDrawing(new TrajectoryDrawing(initial, robot, trajectory, null));
        trajectoryViewer.saveImage();
        //animateViewer.startAnimate();
        //viewer.produceImages(initial, robot, trajectory, L);
	}
	
	/**
	 * the driver to create GUI
	 */
	private static void createAndShowGUI() {
        //Create and set up the window.
        JFrame viewer = new Viewer("Viewer");
        
        viewer.setSize(VIEWER_WIDTH + SELECTION_WIDTH, VIEWER_HEIGHT + BOUNDARY_HEIGHT);
        viewer.setVisible(true);
        viewer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

	/**
	 * @param args
	 */
	public static void main(String[] args) {
        //Schedule a job for the event-dispatching thread:
        //creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
			public void run() {
                createAndShowGUI();
            }
        });
    }
	
	public void setRobot(Robot robot) {
		this.robot = robot;
		System.out.println("Change robot");
	}
	
	public void setQs(Configuration qs) {
		this.qs = qs;
		System.out.println("Change qs");
	}
	
	private class SelectionPane extends JPanel {
		private JTextField xCoordinate = new JTextField(String.format("%f", qs.getX()));
		private JTextField yCoordinate = new JTextField(String.format("%f", qs.getY()));
		private JTextField theta = new JTextField(String.format("%f", qs.getTheta()));
		{
			xCoordinate.addActionListener(e -> {
				Double x = Double.parseDouble(xCoordinate.getText());
				setQs(new Configuration(x, qs.getY(), qs.getTheta()));
				updateTrajectory();
			});
			yCoordinate.addActionListener(e -> {
				Double y = Double.parseDouble(yCoordinate.getText());
				setQs(new Configuration(qs.getX(), y, qs.getTheta()));
				updateTrajectory();
			});
			theta.addActionListener(e-> {
				Double th = Double.parseDouble(theta.getText());
				setQs(new Configuration(qs.getX(), qs.getY(), th));
				updateTrajectory();
			});
		}
		
		private JComboBox<String> robotNameChooser = new JComboBox<>(ROBOT_NAMES);
		{
			robotNameChooser.addActionListener(e -> {
				String robotName = (String)robotNameChooser.getSelectedItem();
				if (robotName != null)
					setRobot(robotMap.get(robotName));
				updateTrajectory();
				
			});
		}
		
		private JComboBox<String> trajectoryChooser = new JComboBox<>();
		{
			trajectoryChooser.addActionListener(e -> {
				String type = (String)trajectoryChooser.getSelectedItem();
				if (type != null)
					updateTrajectory(type);
			});
		}
		
		private JComboBox<String> synthesisChooser = new JComboBox<>(typeMap.keySet().toArray(new String[typeMap.size()]));
		{
			synthesisChooser.addActionListener(e -> {
				String type = (String)synthesisChooser.getSelectedItem();
				if (type != null && typeMap.get(type) != null) {
					synthesisViewer.setType(typeMap.get(type));
					tabbedPane.setSelectedComponent(synthesisViewer);
					synthesisViewer.repaint();
				}
			});
		}
		
		private JButton plan = new JButton("Plan");
		{
			plan.addActionListener(e -> {
				updateTrajectory();
			});
		}
		
		private JButton animate = new JButton("Animate");
		{
			animate.addActionListener(e -> {
				TrajectoryInfo sol = obFreePlanner.getSolution((String)trajectoryChooser.getSelectedItem());
				animateViewer.setDrawing(new TrajectoryDrawing(qs, robot, sol.getTrajectory(), sol.getControlLine()));
				animateViewer.startAnimate();
				tabbedPane.setSelectedComponent(animateViewer);
			});
		}
		
		private JTextField xLB = new JTextField(String.format("%f", rangeX.getBegin()));
		private JTextField xUB = new JTextField(String.format("%f", rangeX.getEnd()));
		private JTextField yLB = new JTextField(String.format("%f", rangeY.getBegin()));
		private JTextField yUB = new JTextField(String.format("%f", rangeY.getEnd()));
		private JTextField thetaTextField = new JTextField(String.format("%f", synthesisTheta));
		private JTextField resolution = new JTextField(String.format("%f", synthesisResolution));
		{
			xLB.addActionListener(e -> {
				Double x = Double.parseDouble(xLB.getText());
				rangeX = new Interval(x, rangeX.getEnd());
			});
			xUB.addActionListener(e -> {
				Double x = Double.parseDouble(xUB.getText());
				rangeX = new Interval(rangeX.getBegin(), x);
			});
			yLB.addActionListener(e -> {
				Double y = Double.parseDouble(yLB.getText());
				rangeY = new Interval(y, rangeY.getEnd());
			});
			yUB.addActionListener(e -> {
				Double y = Double.parseDouble(yUB.getText());
				rangeY = new Interval(rangeY.getBegin(), y);
			});
			thetaTextField.addActionListener(e -> {
				Double theta = Double.parseDouble(thetaTextField.getText());
				synthesisTheta = theta;
			});
			resolution.addActionListener(e -> {
				Double res = Double.parseDouble(resolution.getText());
				synthesisResolution = res;
			});
		}
		
		private JButton synthesis = new JButton("Synthesis");
		{
			synthesis.addActionListener(e -> {
				Synthesis synthesis = new Synthesis.SynthesisBuilder(robot).setX(rangeX.getBegin(), rangeX.getEnd())
				      	                                                   .setY(rangeY.getBegin(), rangeY.getEnd())
				   		                                                   .setTheta(synthesisTheta)
				   		                                                   .setResolution(synthesisResolution)
				   		                                                   .build();
				synthesisViewer.setDrawing(new SynthesisDrawing(synthesis));
				tabbedPane.setSelectedComponent(synthesisViewer);
				synthesisViewer.setType(Drawing.UNFILTER);
				synthesisViewer.repaint();
			});
		}
		
		private void updateTrajectory() {
			TrajectoryInfo sol = obFreePlanner.solve(robot.getControlSet(), new Transformation(qs));
			trajectoryViewer.setDrawing(new TrajectoryDrawing(qs, robot, sol.getTrajectory(), sol.getControlLine()));
			trajectoryChooser.removeAllItems();
			trajectoryChooser.addItem("Optimal");
			for (int i = 0; i < TRAJECTORY_TYPES.length; ++i) {
				String type = TRAJECTORY_TYPES[i];
				if (obFreePlanner.getSolution(type).isValid())
					trajectoryChooser.addItem(type);
			}
			tabbedPane.setSelectedComponent(trajectoryViewer);
			trajectoryViewer.repaint();
		}
		
		private void updateTrajectory(String type) {
			TrajectoryInfo sol = obFreePlanner.getSolution(type);
			trajectoryViewer.setDrawing(new TrajectoryDrawing(qs, robot, sol.getTrajectory(), sol.getControlLine()));
			System.out.println(sol.getTrajectory());
			trajectoryViewer.repaint();
			tabbedPane.setSelectedComponent(trajectoryViewer);
		}
		
		public SelectionPane() {
			this.setLayout(new GridBagLayout());
			GridBagConstraints c = new GridBagConstraints();
			c.fill = GridBagConstraints.HORIZONTAL;
			c.gridx = 0;
			c.gridy = 0;
			c.gridwidth = 2;
			add(new JLabel("Robot"), c);
			
			++c.gridy;
			add(robotNameChooser, c);
			
			++c.gridy;
			add(new JLabel("Initial Configuration"), c);
			
			c.gridwidth = 1;
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("x:"), c);
			c.gridx = 1;
			add(xCoordinate, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("y: "), c);
			c.gridx = 1;
			add(yCoordinate, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("Θ: "), c);
			c.gridx = 1;
			add(theta, c);
			
			++c.gridy;
			c.gridx = 0;
			c.gridwidth = 2;
			add(plan, c);
					
			
			++c.gridy;
			add(trajectoryChooser, c);
			
			++c.gridy;
			c.gridx = 0;
			c.gridwidth = 2;
			add(animate, c);
			
			++c.gridy;
			c.gridwidth = 2;
			add(new JLabel("Synthesis"), c);
			
			c.gridwidth = 1;
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("x lower bound:"), c);
			c.gridx = 1;
			add(xLB, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("x upper bound:"), c);
			c.gridx = 1;
			add(xUB, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("y lower bound:"), c);
			c.gridx = 1;
			add(yLB, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("y upper bound:"), c);
			c.gridx = 1;
			add(yUB, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("Θ: "), c);
			c.gridx = 1;
			add(thetaTextField, c);
			
			++c.gridy;
			c.gridx = 0;
			add(new JLabel("Resolution: "), c);
			c.gridx = 1;
			add(resolution, c);
			
			++c.gridy;
			c.gridx = 0;
			c.gridwidth = 2;
			add(synthesis, c);
			
			++c.gridy;
			add(synthesisChooser, c);
		}
	}
}
