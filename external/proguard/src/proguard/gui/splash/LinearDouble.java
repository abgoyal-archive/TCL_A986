
package proguard.gui.splash;

public class LinearDouble implements VariableDouble
{
    private final double fromValue;
    private final double toValue;
    private final Timing timing;


    /**
     * Creates a new LinearDouble.
     * @param fromValue the value that corresponds to a timing of 0.
     * @param toValue   the value that corresponds to a timing of 1.
     * @param timing    the applied timing.
     */
    public LinearDouble(double fromValue, double toValue, Timing timing)
    {
        this.fromValue = fromValue;
        this.toValue   = toValue;
        this.timing    = timing;
    }


    // Implementation for VariableDouble.

    public double getDouble(long time)
    {
        return fromValue + timing.getTiming(time) * (toValue - fromValue);
    }
}
