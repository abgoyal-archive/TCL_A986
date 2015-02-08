
package proguard.gui.splash;

public class SmoothTiming implements Timing
{
    private final long fromTime;
    private final long toTime;


    /**
     * Creates a new SmoothTiming.
     * @param fromTime the time at which the timing starts ramping up from 0.
     * @param toTime   the time at which the timing stops ramping up at 1.
     */
    public SmoothTiming(long fromTime, long toTime)
    {
        this.fromTime = fromTime;
        this.toTime   = toTime;
    }


    // Implementation for Timing.

    public double getTiming(long time)
    {
        if (time <= fromTime)
        {
            return 0.0;
        }

        if (time >= toTime)
        {
            return 1.0;
        }

        // Compute the linear interpolation.
        double timing = (double) (time - fromTime) / (double) (toTime - fromTime);

        // Smooth the interpolation at the ends.
        return timing * timing * (3.0 - 2.0 * timing);
    }
}
