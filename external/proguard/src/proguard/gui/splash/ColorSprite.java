
package proguard.gui.splash;

import java.awt.*;

public class ColorSprite implements Sprite
{
    private final VariableColor color;
    private final Sprite        sprite;


    /**
     * Creates a new ColorSprite.
     * @param color  the variable color of the given sprite.
     * @param sprite the sprite that will be colored and painted.
     */
    public ColorSprite(VariableColor color,
                       Sprite        sprite)
    {
        this.color  = color;
        this.sprite = sprite;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        // Save the old color.
        Color oldColor = graphics.getColor();

        // Set the new color.
        graphics.setColor(color.getColor(time));

        // Paint the actual sprite.
        sprite.paint(graphics, time);

        // Restore the old color.
        graphics.setColor(oldColor);
    }
}
