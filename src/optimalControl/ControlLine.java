package optimalControl;

/**
 * @author yu-hanlyu
 *
 */
public class ControlLine {
	public static final ControlLine NULL_LINE = new ControlLine(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
	private double kx; // kx * kx + ky * ky = 1
	private double ky;
	private double ktheta;
	private double H;
	
	/**
	 * @param arg_kx: kx
	 * @param arg_ky: ky
	 * @param arg_ktheta: ktheta
	 * @param arg_H: Hamiltonian value
	 */
	public ControlLine(double arg_kx, double arg_ky, double arg_ktheta, double arg_H) {
		kx = arg_kx;
		ky = arg_ky;
		ktheta = arg_ktheta;
		H = arg_H;
	}
	
	/**
	 * @return return kx
	 */
	public double getKx() {
		return kx;
	}
	
	/**
	 * @return return ky
	 */
	public double getKy() {
		return ky;
	}
	
	/**
	 * @return return ktheta
	 */
	public double getKtheta() {
		return ktheta;
	}
	
	/**
	 * @return return Hamiltonian value
	 */
	public double getH() {
		return H;
	}
	
	/**
	 * Test whether a control line is valid
	 * @return true is the control line is valid
	 */
	public boolean isValid() {
		return Double.isFinite(kx) && Double.isFinite(ky) && Double.isFinite(ktheta) && Double.isFinite(H);
    }
	
	/**
	 * String representation of the control line
	 */
	@Override
	public String toString() {
		return String.format("%07f", kx) + " " + String.format("%07f", ky) + " " +
		       String.format("%07f", ktheta) + " " + String.format("%07f", H);
	}
}
