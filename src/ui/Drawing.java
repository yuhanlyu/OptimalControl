package ui;
import java.awt.Graphics2D;


public abstract class Drawing {
	public static final Drawing NULL_DRAWING = new NullDrawing();
	public static final int UNFILTER = 0;
	public abstract void draw(Graphics2D g);
	public void incrementTime(double step) {}
	protected int type = UNFILTER;
	
	private static class NullDrawing extends Drawing {
		@Override
		public void draw(Graphics2D g) {}
	}
	
	/**
	 * Set the type
	 * @param type
	 */
	public void setType(int type) {
		this.type = type;
	}
}
