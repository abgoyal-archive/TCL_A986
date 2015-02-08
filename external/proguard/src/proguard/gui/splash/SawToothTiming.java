
package proguard.gui.splash;

public class SawToothTiming implements Timing
{
    private final long period;
    private final long phase;


    /**
     * Creates a new SawToothTiming.
     * @param period the time period for a full cycle.
     * @param phase  the phase of the cycle, which is added to the actual time.
     */
    public SawToothTiming(long period, long phase)
    {
        this.period = period;
        this.phase  = phase;
    }


    // Implementation for Timing.

    public double getTiming(long time)
    {
        // Compute the translated and scaled saw-tooth function.
        return (double)((time + phase) % period) / (double)period;
    }
}
