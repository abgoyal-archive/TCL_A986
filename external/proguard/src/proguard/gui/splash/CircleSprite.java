
package proguard.gui.splash;

import java.awt.*;

public class CircleSprite implements Sprite
{
    private final boolean     filled;
    private final VariableInt x;
    private final VariableInt y;
    private final VariableInt radius;


    /**
     * Creates a new CircleSprite.
     * @param filled specifies whether the rectangle should be filled.
     * @param x      the variable x-coordinate of the center of the circle.
     * @param y      the variable y-coordinate of the center of the circle.
     * @param radius the variable radius of the circle.
     */
    public CircleSprite(boolean     filled,
                        VariableInt x,
                        VariableInt y,
                        VariableInt radius)
    {
        this.filled = filled;
        this.x      = x;
        this.y      = y;
        this.radius = radius;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        int xt = x.getInt(time);
        int yt = y.getInt(time);
        int r  = radius.getInt(time);

        if (filled)
        {
            graphics.fillOval(xt - r, yt - r, 2 * r, 2 * r);
        }
        else
        {
            graphics.drawOval(xt - r, yt - r, 2 * r, 2 * r);
        }
    }
}
