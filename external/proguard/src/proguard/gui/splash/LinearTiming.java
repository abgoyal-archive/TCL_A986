
package proguard.gui.splash;

public class LinearTiming implements Timing
{
    private final long fromTime;
    private final long toTime;


    /**
     * Creates a new LinearTiming.
     * @param fromTime the time at which the timing starts ramping up from 0.
     * @param toTime   the time at which the timing stops ramping up at 1.
     */
    public LinearTiming(long fromTime, long toTime)
    {
        this.fromTime = fromTime;
        this.toTime   = toTime;
    }


    // Implementation for Timing.

    public double getTiming(long time)
    {
        // Compute the clamped linear interpolation.
        return time <= fromTime ? 0.0 :
               time >= toTime   ? 1.0 :
                                  (double)(time - fromTime) / (double)(toTime - fromTime);
    }
}
