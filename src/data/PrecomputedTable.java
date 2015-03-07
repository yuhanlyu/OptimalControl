package data;

import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.xml.stream.XMLOutputFactory;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamWriter;

import optimalControl.Configuration;
import optimalControl.Control;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;

import rigidBody2DCostlySwitch.CostlyPlanner;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import robotModel.DubinsCar;
import robotModel.Robot;

public class PrecomputedTable {
	private static final double SWITCH_COST = 1.0;
	private Robot robot;
	private double xLowerBound, xUpperBound;
	private double yLowerBound, yUpperBound;
	private double resolution;
	private Map<Configuration, List<Trajectory>> mapping = new HashMap<>();
	
	public PrecomputedTable(TableBuilder builder) {
		this.robot = builder.robot;
		this.xLowerBound = builder.xLowerBound;
		this.xUpperBound = builder.xUpperBound;
		this.yLowerBound = builder.yLowerBound;
		this.yUpperBound = builder.yUpperBound;
		this.resolution = builder.resolution;
		compute();
	}
	
	private void compute() {
		try {
			XMLOutputFactory xof = XMLOutputFactory.newInstance();
			XMLStreamWriter xmlsw = xof.createXMLStreamWriter(new FileWriter("table.xml"));
			xmlsw.writeStartDocument("utf-8", "1.0");
 
			// root elements
			createRobot(xmlsw);
			
			createTable(xmlsw);
				 
			xmlsw.writeEndDocument();
			xmlsw.flush();
			xmlsw.close();
			System.out.println("File saved!");
		} catch (XMLStreamException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private void createTrajectory(XMLStreamWriter xmlsw, Trajectory trajectory) throws XMLStreamException {
		for (int i = 0; i < trajectory.size(); ++i) {
			Control u = trajectory.getControl(i);
			xmlsw.writeStartElement("control");
			xmlsw.writeAttribute("index", new Integer(i).toString());
			xmlsw.writeAttribute("vx", new Double(u.getVx()).toString());
			xmlsw.writeAttribute("vy", new Double(u.getVy()).toString());
			xmlsw.writeAttribute("omega", new Double(u.getOmega()).toString());
			xmlsw.writeAttribute("duration", new Double(trajectory.getDuration(i)).toString());
			xmlsw.writeEndElement();
		}
	}
	
	private void createConfiguration(XMLStreamWriter xmlsw, Configuration qs) throws XMLStreamException {
		ControlSet U = robot.getControlSet();
		Transformation Ts = new Transformation(qs);
		CostlyPlanner planner = new CostlyPlanner(SWITCH_COST);
		List<TrajectoryInfo> solutions = planner.getAllTrajectories(U, Ts);
		//List<Trajectory> sols = new ArrayList<>();
		//for (TrajectoryInfo solution : solutions) {
		//	sols.add(solution.getTrajectory());
		//}
		//mapping.put(qs,  sols);

		for (TrajectoryInfo solution : solutions) {
			Trajectory traj = solution.getTrajectory();
			xmlsw.writeStartElement("trajectory");
			xmlsw.writeAttribute("actions", new Integer(traj.size()).toString());
			xmlsw.writeAttribute("duration", new Double(traj.totalTime()).toString());
			createTrajectory(xmlsw, traj);
			xmlsw.writeEndElement();
		}
	}
	
	private void createTable(XMLStreamWriter xmlsw) throws XMLStreamException {
		int totalNumber = (int)((xUpperBound - xLowerBound) * (yUpperBound - yLowerBound) * (2.0 * Math.PI) / (resolution * resolution * resolution));
		System.out.println(totalNumber);
		int currentNumber = 0;
		for (double x = xLowerBound; x < xUpperBound; x += resolution) {
			for (double y = yLowerBound; y < yUpperBound; y += resolution) {
				for (double theta = 0; theta < 2.0 * Math.PI; theta += resolution) {
					xmlsw.writeStartElement("configuration");
					Configuration qs = new Configuration(x, y, theta);
					xmlsw.writeAttribute("x", new Double(x).toString());
					xmlsw.writeAttribute("y", new Double(y).toString());
					xmlsw.writeAttribute("theta", new Double(theta).toString());
					createConfiguration(xmlsw, qs);
					++currentNumber;
					System.out.println("Progress = " + currentNumber + "/" + totalNumber);
					xmlsw.writeEndElement();
				}
			}
			
		}
	}
	
	private void createRobot(XMLStreamWriter xmlsw) throws XMLStreamException {
		xmlsw.writeStartElement("robot");
		xmlsw.writeAttribute("size", new Integer(robot.getControlSet().size()).toString());
		for (int i = 0; i < robot.getControlSet().size(); ++i) {
			xmlsw.writeStartElement("control");
			xmlsw.writeAttribute("index", new Integer(i).toString());
			Control u = robot.getControl(i);
			xmlsw.writeAttribute("vx", new Double(u.getVx()).toString());
			xmlsw.writeAttribute("vy", new Double(u.getVy()).toString());
			xmlsw.writeAttribute("omega", new Double(u.getOmega()).toString());
			xmlsw.writeEndElement();
		}
		xmlsw.writeEndElement();
	}
		
	
	/**
	 * Builder for synthesis
	 * @author yu-hanlyu
	 *
	 */
	public static class TableBuilder {
		private Robot robot;
		private double xLowerBound, xUpperBound;
		private double yLowerBound, yUpperBound;
		private double resolution;
		
		/**
		 * Constructor
		 * @param robot
		 */
		public TableBuilder(Robot robot) {
			this.robot = robot;
		}
		
		/**
		 * Set the lower and upper bounds for the x-coordinate
		 * @param xLowerBound
		 * @param xUpperBound
		 * @return
		 */
		public TableBuilder setX(double xLowerBound, double xUpperBound) {
			this.xLowerBound = xLowerBound;
			this.xUpperBound = xUpperBound;
			return this;
		}
		
		/**
		 * Set the lower and upper bound for the y-coordinate
		 * @param yLowerBound
		 * @param yUpperBound
		 * @return
		 */
		public TableBuilder setY(double yLowerBound, double yUpperBound) {
			this.yLowerBound = yLowerBound;
			this.yUpperBound = yUpperBound;
			return this;
		}
		
		/**
		 * Set the resolution
		 * @param resolution
		 * @return
		 */
		public TableBuilder setResolution(double resolution) {
			this.resolution = resolution;
			return this;
		}
		
		/**
		 * build
		 * @return
		 */
		public PrecomputedTable build() {
			return new PrecomputedTable(this);
		}
	}
	
	public static void main(String argv[]) {
		PrecomputedTable table = new TableBuilder(new DubinsCar()).setX(-1, 1)
				                                                  .setY(-1, 1)
				                                                  .setResolution(1)
				                                                  .build();
	}
}
