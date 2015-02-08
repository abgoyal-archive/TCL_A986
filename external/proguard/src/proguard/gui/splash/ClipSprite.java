
package proguard.gui.splash;

import java.awt.*;

public class ClipSprite implements Sprite
{
    private final VariableColor insideClipColor;
    private final VariableColor outsideClipColor;
    private final Sprite        clipSprite;
    private final Sprite        sprite;


    /**
     * Creates a new ClipSprite.
     * @param insideClipColor  the background color inside the clip sprite.
     * @param outsideClipColor the background color outside the clip sprite.
     * @param clipSprite       the clip Sprite.
     * @param sprite           the clipped Sprite.
     */
    public ClipSprite(VariableColor insideClipColor,
                      VariableColor outsideClipColor,
                      Sprite        clipSprite,
                      Sprite        sprite)
    {
        this.insideClipColor  = insideClipColor;
        this.outsideClipColor = outsideClipColor;
        this.clipSprite       = clipSprite;
        this.sprite           = sprite;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        // Clear the background.
        Color outsideColor = outsideClipColor.getColor(time);
        Rectangle clip = graphics.getClipBounds();
        graphics.setPaintMode();
        graphics.setColor(outsideColor);
        graphics.fillRect(0, 0, clip.width, clip.height);

        // Draw the sprite in XOR mode.
        OverrideGraphics2D g = new OverrideGraphics2D((Graphics2D)graphics);
        Color insideColor = insideClipColor.getColor(time);
        g.setOverrideXORMode(insideColor);
        sprite.paint(g, time);
        g.setOverrideXORMode(null);

        // Clear the clip area.
        g.setOverrideColor(insideColor);
        clipSprite.paint(g, time);
        g.setOverrideColor(null);

        // Draw the sprite in XOR mode.
        g.setOverrideXORMode(insideColor);
        sprite.paint(g, time);
        g.setOverrideXORMode(null);
    }
}
