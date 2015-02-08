
package proguard.gui.splash;

import java.awt.*;

public class CompositeSprite implements Sprite
{
    private final Sprite[] sprites;


    /**
     * Creates a new CompositeSprite.
     * @param sprites the array of Sprite objects to which the painting will
     *                be delegated, starting with the first element.
     */
    public CompositeSprite(Sprite[] sprites)
    {
        this.sprites = sprites;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        // Draw the sprites.
        for (int index = 0; index < sprites.length; index++)
        {
            sprites[index].paint(graphics, time);
        }
    }
}
