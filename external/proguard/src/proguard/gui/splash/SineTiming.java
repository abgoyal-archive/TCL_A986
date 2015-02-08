
package proguard.gui.splash;

public class SineTiming implements Timing
{
    private final long period;
    private final long phase;


    /**
     * Creates a new SineTiming.
     * @param period the time period for a full cycle.
     * @param phase  the phase of the cycle, which is added to the actual time.
     */
    public SineTiming(long period, long phase)
    {
        this.period = period;
        this.phase  = phase;
    }


    // Implementation for Timing.

    public double getTiming(long time)
    {
        // Compute the translated and scaled sine function.
        return 0.5 + 0.5 * Math.sin(2.0 * Math.PI * (time + phase) / period);
    }
}
