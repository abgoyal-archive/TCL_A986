
package proguard.gui.splash;

import java.awt.*;

public class FontSprite implements Sprite
{
    private final VariableFont font;
    private final Sprite       sprite;


    /**
     * Creates a new FontSprite.
     * @param font   the variable Font of the given sprite.
     * @param sprite the sprite that will be provided of a font and painted.
     */
    public FontSprite(VariableFont font,
                      Sprite       sprite)
    {
        this.font   = font;
        this.sprite = sprite;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        // Save the old font.
        Font oldFont = graphics.getFont();

        // Set the new font.
        graphics.setFont(font.getFont(time));

        // Paint the actual sprite.
        sprite.paint(graphics, time);

        // Restore the old font.
        graphics.setFont(oldFont);
    }
}
