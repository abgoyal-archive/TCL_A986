
package proguard.gui.splash;

public class LinearInt implements VariableInt
{
    private final int    fromValue;
    private final int    toValue;
    private final Timing timing;


    /**
     * Creates a new LinearInt.
     * @param fromValue the value that corresponds to a timing of 0.
     * @param toValue   the value that corresponds to a timing of 1.
     * @param timing    the applied timing.
     */
    public LinearInt(int fromValue, int toValue, Timing timing)
    {
        this.fromValue = fromValue;
        this.toValue   = toValue;
        this.timing    = timing;
    }


    // Implementation for VariableInt.

    public int getInt(long time)
    {
        return (int) (fromValue + timing.getTiming(time) * (toValue - fromValue));
    }
}
