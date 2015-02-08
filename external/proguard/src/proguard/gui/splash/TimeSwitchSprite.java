
package proguard.gui.splash;

import java.awt.*;

public class TimeSwitchSprite implements Sprite
{
    private final long   onTime;
    private final long offTime;
    private final Sprite sprite;


    /**
     * Creates a new TimeSwitchSprite for displaying a given Sprite starting at
     * a given time.
     * @param onTime the start time.
     * @param sprite the toggled Sprite.
     */
    public TimeSwitchSprite(long onTime, Sprite sprite)
    {
        this(onTime, 0L, sprite);
    }


    /**
     * Creates a new TimeSwitchSprite for displaying a given Sprite in a given
     * time interval.
     * @param onTime  the start time.
     * @param offTime the stop time.
     * @param sprite  the toggled Sprite.
     */
    public TimeSwitchSprite(long onTime, long offTime, Sprite sprite)
    {
        this.onTime  = onTime;
        this.offTime = offTime;
        this.sprite  = sprite;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        if (time >= onTime && (offTime <= 0 || time <= offTime))
        {
            sprite.paint(graphics, time - onTime);
        }

    }
}
