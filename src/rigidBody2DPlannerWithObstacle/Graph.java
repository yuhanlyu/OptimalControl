package rigidBody2DPlannerWithObstacle;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import optimalControl.ControlLine;
import optimalControl.Transformation;
import optimalControl.Trajectory;

/**
 * A graph class storing information of trajectories satisfying necessary conditions
 * @author yu-hanlyu
 *
 */
public class Graph {
	private Map<Transformation, Vertex> vertices = new HashMap<>();
	
	public Graph() {
	}
	
	public Collection<Vertex> getVertices() {
		return vertices.values();
	}
	
	public Vertex addVertex(Transformation T) {
		if (vertices.get(T) != null)
			throw new IllegalArgumentException("Duplicate node");
		Vertex node = new Vertex(T);
		vertices.put(T, node);
		return node;
	}
	
	/**
	 * A class storing information about Transformation
	 * @author yu-hanlyu
	 *
	 */
	public class Vertex {
		private Transformation T;
		private Map<Vertex, Edge> edges = new HashMap<>();
		
		public Vertex(Transformation T) {
			this.T= T;
		}
		
		public Transformation getTransformation() {
			return T;
		}
		
		public void addTrajectory(Vertex destination, Trajectory trajectory, ControlLine controlLine) {
			if (edges.get(destination) != null) {
				throw new IllegalArgumentException("Duplicate edge");
			}
			edges.put(destination, new Edge(this, destination, trajectory, controlLine));
		}
		
		public Collection<Edge> getEdges() {
			return edges.values();
		}
	}
	
	/**
	 * A class storing information of trajectories between two configurations
	 * @author yu-hanlyu
	 *
	 */
	public class Edge {
		private Vertex source;
		private Vertex destination;
		private Trajectory trajectory;
		private ControlLine controlLine;
		
		public Edge(Vertex source, Vertex destination, Trajectory trajectory, ControlLine controlLine) {
			this.source = source;
			this.destination = destination;
			this.trajectory = trajectory;
			this.controlLine = controlLine;
		}
		
		public Trajectory getTrajectory() {
			return trajectory;
		}
		
		public ControlLine getControlLine() {
			return controlLine;
		}
		
		public Vertex getSource() {
			return source;
		}
		
		public Vertex getDestination() {
			return destination;
		}
	}
}
