package optimalControl;

/**
 * An interface for distance-time and Lipschitz constant functor
 * @author yu-hanlyu
 *
 */
public abstract class DTFunctor {
	public abstract DistanceTime getDistanceTime(double H);
	
	public abstract DistanceTimeL getL(double H);
	
	public static final DTFunctor create(Control pre, Control current, Control next) {
		return current.isTranslation() ? new TranslationDTFunctor(pre, current, next)
		                               : new RotationDTFunctor(pre, current, next);
	}
}
