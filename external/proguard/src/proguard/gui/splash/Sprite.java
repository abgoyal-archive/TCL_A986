
package proguard.gui.splash;

import java.awt.*;

public interface Sprite
{
    /**
     * Paints the object.
     *
     * @param graphics the Graphics to paint on.
     * @param time     the time since the start of the animation, expressed in
     *                  milliseconds.
     */
    public void paint(Graphics graphics, long time);
}
