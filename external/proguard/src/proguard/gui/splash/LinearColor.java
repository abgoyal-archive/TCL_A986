
package proguard.gui.splash;

import java.awt.*;

public class LinearColor implements VariableColor
{
    private final Color  fromValue;
    private final Color  toValue;
    private final Timing timing;

    private double cachedTiming = -1.0;
    private Color  cachedColor;


    /**
     * Creates a new LinearColor.
     * @param fromValue the value that corresponds to a timing of 0.
     * @param toValue   the value that corresponds to a timing of 1.
     * @param timing    the applied timing.
     */
    public LinearColor(Color fromValue, Color toValue, Timing timing)
    {
        this.fromValue = fromValue;
        this.toValue   = toValue;
        this.timing    = timing;
    }


    // Implementation for VariableColor.

    public Color getColor(long time)
    {
        double t = timing.getTiming(time);
        if (t != cachedTiming)
        {
            cachedTiming = t;
            cachedColor =
                t == 0.0 ? fromValue :
                t == 1.0 ? toValue   :
                           new Color((int)(fromValue.getRed()   + t * (toValue.getRed()   - fromValue.getRed())),
                                     (int)(fromValue.getGreen() + t * (toValue.getGreen() - fromValue.getGreen())),
                                     (int)(fromValue.getBlue()  + t * (toValue.getBlue()  - fromValue.getBlue())));
        }

        return cachedColor;
    }
}
