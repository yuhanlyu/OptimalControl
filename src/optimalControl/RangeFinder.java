package optimalControl;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * A class that computes the range of the Hamiltonian values 
 * @author yu-hanlyu
 *
 */
public class RangeFinder {
	private ControlSet U;
	private Transformation Ts;
	private Control us;
	private Transformation Tf;
	private Control uf;
	private ControlLineFactory factory;
	
	/**
	 * 
	 * @param U
	 * @param Ts
	 * @param us
	 * @param Tf
	 * @param uf
	 */
	public RangeFinder(ControlSet U, Transformation Ts, Control us, 
			                         Transformation Tf, Control uf, ControlLineFactory factory) {
		this.U = U;
		this.Ts = Ts;
		this.us = us;
		this.Tf = Tf;
		this.uf = uf;
		this.factory = factory;
	}
	
	/**
	 * Return the range of the Hamiltonian which controls are maximizing
	 * @param positive sign of the control line
	 * @return the range of Hamiltonian
	 */
	public Interval getRange() {
		return range(Ts, us).intersect(range(Tf, uf));
	}
	
	/**
	 * Return the range of the Hamiltonian which u is maximizing with respect to U at T
	 * @param U control set
	 * @param T the configuration
	 * @param u control
	 * @param positive sign of the control line
	 * @return the range of the Hamiltonian which u is maximizing with respect to U at T
	 */
	private Interval range(Transformation T, Control u) {
		List<Double> critHs = enumCritHs(T, u);
		int begin = find(T, u, critHs, true);
		int end = find(T, u, critHs, false);
		if (begin == critHs.size() - 1)
			return Interval.EMPTY_INTERVAL;
		return new Interval(critHs.get(begin), critHs.get(end));
	}
	
	/**
	 * Enumerate the list of H values that possibly are the end points of the range
	 * @param U control set
	 * @param T the configuration
	 * @param u control
	 * @param positive sign of the control line
	 * @return the list of H values that possibly are the end points of the range
	 */
	private List<Double> enumCritHs(Transformation T, Control u) {
		double ub = getUpperBound(u);
		return Stream.concat(U.controlStream()
                              .map(ui -> u.equals(ui) ? Double.NaN : 
                            	                        factory.switchPointOnLine(T, u, ui)), 
                             Stream.of(new Double(0.0), new Double(ub)))
		             .filter(H -> Double.isFinite(H) && H <= ub)
		             .sorted()
                     .collect(Collectors.toList());				
	}
	
	/**
	 * Compute the upper bound of Hamiltonian
	 * @param u
	 * @return
	 */
	private double getUpperBound(Control u) {
		double ub = U.getUpperBound();
		if (u.isTranslation())
			ub = Double.min(u.getVelocity(), ub);
		return Double.min(ub, factory.getUpperBoundOfH());
	}
	
	/**
	 * Find the index of the boundary critical values
	 * @param T the configuration
	 * @param u the control 
	 * @param positive the sign of the control line
	 * @param critHs possible end points of critical values
	 * @param forward if is true, then find from begin, otherwise find from the end
	 * @return the index of theboundary
	 */
	private int find(Transformation T, Control u, List<Double> critHs, boolean forward) {
		int begin = forward ? 0 : critHs.size() - 1;
		int end = forward ? critHs.size() - 1 : 0;
		int step = forward ? 1 : -1;
		for (int i = begin; i != end; i += step) {
			double H = (critHs.get(i) + critHs.get(i + step)) * 0.5;
			ControlLine controlLine = factory.getControlLine(H);
			Transformation TLR = (new Transformation(controlLine)).transform(T);
			if (U.isSustainable(TLR, u))
				return i;
		}
		return end;
	}
}
