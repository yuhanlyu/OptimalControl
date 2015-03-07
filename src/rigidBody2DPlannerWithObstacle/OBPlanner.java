package rigidBody2DPlannerWithObstacle;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.PriorityQueue;
import java.util.stream.IntStream;

import optimalControl.ControlLine;
import optimalControl.ControlSet;
import optimalControl.Trajectory;
import optimalControl.Transformation;
import obstacleModel.Environment;
import rigidBody2DCostlySwitch.CostlyGenericAllSolver;
import rigidBody2DCostlySwitch.CostlyGenericSolver;
import rigidBody2DCostlySwitch.CostlyPlanner;
import rigidBody2DCostlySwitch.CostlySingularAllSolver;
import rigidBody2DCostlySwitch.CostlySingularSolver;
import rigidBody2DCostlySwitch.CostlyTGTAllSolver;
import rigidBody2DCostlySwitch.CostlyTGTSolver;
import rigidBody2DCostlySwitch.CostlyWhirlAllSolver;
import rigidBody2DCostlySwitch.CostlyWhirlSolver;
import rigidBody2DFreeSwitch.FeasibleSolver;
import rigidBody2DFreeSwitch.FreePlanner;
import rigidBody2DFreeSwitch.GenericAllSolver;
import rigidBody2DFreeSwitch.GenericSolver;
import rigidBody2DFreeSwitch.LipschitzianMinimizer;
import rigidBody2DFreeSwitch.SingularAllSolver;
import rigidBody2DFreeSwitch.SingularSolver;
import rigidBody2DFreeSwitch.TGTSolver;
import rigidBody2DFreeSwitch.TrajectoryInfo;
import rigidBody2DFreeSwitch.UniformSampleMinimizer;
import rigidBody2DFreeSwitch.WhirlSolver;
import rigidBody2DPlannerWithObstacle.Graph.Edge;
import rigidBody2DPlannerWithObstacle.Graph.Vertex;

/**
 * A planner for environments with obstacles
 * @author yu-hanlyu
 *
 */
public class OBPlanner {
	private static final double DELTA = 1;
	private static final boolean PARALLEL = false;
	private static final double SWITCH_COST = 1.0;
	private ControlSet U;
	private Environment env;
	private Graph graph = new Graph();
	private Point2D fakeGoal;
	private double vMax;
	
	/**
	 * Constructor
	 * @param U
	 * @param env
	 */
	public OBPlanner(ControlSet U, Environment env) {
		this.U = U;
		this.env = env;
		this.vMax = U.maxVelocity();
		System.out.println("Building graph");
		buildGraph();
		System.out.println("Graph is built, #vertices = " + graph.getVertices().size());
	}
	
	/**
	 * Find an approximately optimal path from Ts to Tf
	 * @param Ts
	 * @param Tf
	 * @return
	 */
	public Trajectory solve(Transformation Ts, Transformation Tf) {
		Vertex begin = graph.addVertex(Ts);
		Vertex end = graph.addVertex(Tf);
		
		Collection<Vertex> vertices = graph.getVertices();
		for (Vertex vertex : vertices) {
			if (begin != vertex) {
				buildEdges(begin, vertex);
				if (end != vertex)
					buildEdges(vertex, end);
			}
		}
		fakeGoal = Tf.toPoint();
		return findPath(begin, end);
	}
	
	/**
	 * Search node for A* search
	 * @author yu-hanlyu
	 *
	 */
	private class SearchNode implements Comparable<SearchNode> {
		private Vertex vertex;
		private SearchNode parent;
		private Trajectory trajectory;
		private double cost;
		
		public SearchNode(Vertex vertex, SearchNode parent, Trajectory trajectory, double costSoFar) {
			this.vertex = vertex;
			this.parent = parent;
			this.trajectory = trajectory;
			this.cost = costSoFar + trajectory.totalTime();
		}
		
		public double getCost() {
			return cost;
		}
		
		public Vertex getVertex() {
			return vertex;
		}
		
		public double getHeuristic() {
			return cost + vertex.getTransformation().toPoint().distance(fakeGoal) / vMax;
		}
		
		public Trajectory backChain() {
			SearchNode node = this;
			Trajectory result = new Trajectory();
			List<SearchNode> path = new ArrayList<>();
			for (; node != null; node = node.parent) {
				path.add(node);
				System.out.println(node.getVertex().getTransformation());
			}
			for (int i = path.size() - 1; i >= 0; --i) {
				result.append(path.get(i).trajectory);
			}
			return result;
		}
		
		@Override
		public int compareTo(SearchNode o) {
			if (getHeuristic() < o.getHeuristic())
				return -1;
			else if (getHeuristic() > o.getHeuristic())
				return 1;
			return 0;
		}
		
		@Override
		public int hashCode() {
			int h1 = vertex.hashCode();
			return h1;
		}
		
		@Override
		public boolean equals(Object o) {
			SearchNode node = (SearchNode)o;
			return vertex == node.vertex;
		}
	}
	
	/**
	 * Find a path by using A* search
	 * @param begin
	 * @param end
	 * @return
	 */
	private Trajectory findPath(Vertex begin, Vertex end) {
		PriorityQueue<SearchNode> queue = new PriorityQueue<>();
		SearchNode root = new SearchNode(begin, null, new Trajectory(), 0.0);
		queue.add(root);
		System.out.println("Searching for path");
		
		while (!queue.isEmpty()) {
			SearchNode node = queue.poll();
			Vertex vertex = node.getVertex();
			if (node.getVertex() == end) {
				System.out.println("Path is found");
				return node.backChain();
			}
			if (node.getHeuristic() > 20.0) {
				System.out.println("Break");
				break;
			}
			for (Edge e : vertex.getEdges()) {
				Trajectory trajectory = e.getTrajectory();
				SearchNode newNode = new SearchNode(e.getDestination(), node, trajectory, node.getCost());
				queue.add(newNode);
			}
		}
		System.out.println("Path not found");
		return null;
	}
	
	/**
	 * Build graph
	 */
	public void buildGraph() {
		List<List<Transformation>> samples = env.generateSamples(DELTA);
		Vertex[][] vertices = new Vertex[samples.size()][];
		for (int i = 0; i < samples.size(); ++i) {
			vertices[i] = new Vertex[samples.get(i).size()];
			for (int j = 0; j < samples.get(i).size(); ++j)
				vertices[i][j] = graph.addVertex(samples.get(i).get(j));
		}
		long startTime = System.nanoTime();
		if (PARALLEL)
			IntStream.range(0, samples.size()).parallel().forEach(i -> buildEdges(vertices[i], vertices));
		else
			IntStream.range(0, samples.size()).forEach(i -> buildEdges(vertices[i], vertices));
		long endTime = System.nanoTime();
		double duration = (endTime - startTime) / 1000000000.0;
		System.out.println("Built graph in " + duration + "seconds.");
	}
	
	private void buildEdges(Vertex[] sourceNodes, Vertex[][] vertices) {
		for (int i = 0; i < vertices.length; ++i) {
			buildEdges(sourceNodes, vertices[i]);
		}
	}
	
	/**
	 * Build edges between from sourceNodes to destNodes
	 * @param sourceNodes
	 * @param destNodes
	 */
	private void buildEdges(Vertex[] sourceNodes, Vertex[] destNodes) {
		for (int i = 0; i < sourceNodes.length; ++i) {
			for (int j = 0; j < destNodes.length; ++j) {
				if (sourceNodes[i] != destNodes[j]) {
					System.out.println("i-th node: " + i + " j-th node: " + j);
					buildEdges(sourceNodes[i], destNodes[j]);
				}
			}
		}
	}
	
	/**
	 * Build edges between two nodes
	 * @param source
	 * @param destination
	 */
	private void buildEdges(Vertex source, Vertex destination) {
		Transformation Ts = source.getTransformation();
		Transformation Tf = destination.getTransformation();
		Transformation T = rotateTfToOrigin(Ts, Tf);
		//FreePlanner planner = new FreePlanner();
		CostlyPlanner planner = new CostlyPlanner(SWITCH_COST);
		List<TrajectoryInfo> sols = planner.getAllTrajectories(U, T);
		buildEdges(source, destination, sols, Ts);
	}
	
	private void buildEdges(Vertex source, Vertex destination, List<TrajectoryInfo> sols, Transformation Ts) {
		Trajectory edge = null;
		ControlLine edgeControlLine = null;
		double minTime = Double.POSITIVE_INFINITY;
		for (TrajectoryInfo sol : sols) {
			Trajectory trajectory = sol.getTrajectory();
			if (trajectory.totalTime() < 0.0001)
				System.out.println("Error " + source.getTransformation() + " " + destination.getTransformation());
			if (Double.isFinite(trajectory.totalTime()) && minTime > trajectory.totalTime() && !env.isCollide(Ts, trajectory)) {
				minTime = trajectory.totalTime();
				edge = trajectory;
				edgeControlLine = sol.getControlLine();
			}
		}
		if (Double.isFinite(minTime))
			source.addTrajectory(destination, edge, edgeControlLine);
	}
	
	/**
	 * After moving Tf to the origin, compute the result of Ts
	 * @param Ts
	 * @param Tf
	 * @return
	 */
	private static Transformation rotateTfToOrigin(Transformation Ts, Transformation Tf) {
		double dx = Ts.getX() - Tf.getX();
		double dy = Ts.getY() - Tf.getY();
		double dtheta = Ts.getTheta() - Tf.getTheta();
		double sin = -Tf.getSin();
		double cos = Tf.getCos();
		double x = cos * dx - sin * dy;
		double y = sin * dx + cos * dy;
		return new Transformation(x, y, Math.sin(dtheta), Math.cos(dtheta));
	}
	
	/**
	 * Return the graph
	 * @return
	 */
	public Graph getGraph() {
		return graph;
	}
}
