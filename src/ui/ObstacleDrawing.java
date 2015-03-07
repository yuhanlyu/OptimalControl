package ui;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import optimalControl.Trajectory;
import optimalControl.Transformation;
import obstacleModel.Environment;
import rigidBody2DPlannerWithObstacle.Graph;
import rigidBody2DPlannerWithObstacle.Graph.Edge;
import rigidBody2DPlannerWithObstacle.Graph.Vertex;


public class ObstacleDrawing extends Drawing {
	private Environment env;
	private Graph graph;
	private List<TrajectoryShape> trajectories = new ArrayList<>();
	
	public ObstacleDrawing(Environment env, Graph graph) {
		this.env = env;
		this.graph = graph;
		//createTrajectoryShapes();
	}
	
	public void addTrajectory(Transformation Ts, Trajectory trajectory) {
		System.out.println(trajectory);
		trajectories.add(new TrajectoryShape(Ts, trajectory));
	}
	
	@Override
	public void draw(Graphics2D g) {
		Color old = g.getColor();
		g.setColor(Color.ORANGE);
		for (TrajectoryShape s : trajectories) {
			s.draw(g);
		}
		/*
		for (Vertex source : graph.getVertices()) {
			Point2D u = source.getTransformation().toPoint();
			for (Edge e : source.getEdges()) {
				Vertex destination = e.getDestination();
				Point2D v = destination.getTransformation().toPoint();
				int x1 = (int)(u.getX() * Viewer.SCALE);
				int y1 = (int)(u.getY() * Viewer.SCALE);
				int x2 = (int)(v.getX() * Viewer.SCALE);
				int y2 = (int)(v.getY() * Viewer.SCALE);
				g.drawLine(x1, y1, x2, y2);
			}
		}*/
		g.setColor(Color.black);
		for (Shape s : env.getShapes())
			g.draw(Viewer.scale.createTransformedShape(s));
		
		g.setColor(old);
	}
	
	private void createTrajectoryShapes() {
		for (Vertex node : graph.getVertices()) {
			for (Edge e : node.getEdges()) {
				Trajectory trajectory = e.getTrajectory();
				trajectories.add(new TrajectoryShape(node.getTransformation(), trajectory));
			}
		}
	}
}
