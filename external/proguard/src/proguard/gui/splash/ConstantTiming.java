
package proguard.gui.splash;

public class ConstantTiming implements Timing
{
    private final double timing;


    /**
     * Creates a new ConstantTiming with a value of 0.
     */
    public ConstantTiming()
    {
        this(0.0);
    }

    /**
     * Creates a new ConstantTiming with a given value.
     * @param timing the constant value of the timing.
     */
    public ConstantTiming(double timing)
    {
        this.timing = timing;
    }


    // Implementation for Timing.

    public double getTiming(long time)
    {
        return timing;
    }
}
