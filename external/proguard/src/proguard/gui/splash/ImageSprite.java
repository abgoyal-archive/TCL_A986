
package proguard.gui.splash;

import java.awt.*;

public class ImageSprite implements Sprite
{
    private final Image          image;
    private final VariableInt    x;
    private final VariableInt    y;
    private final VariableDouble scaleX;
    private final VariableDouble scaleY;


    /**
     * Creates a new ImageSprite.
     * @param image  the Image to be painted.
     * @param x      the variable x-coordinate of the upper-left corner of the image.
     * @param y      the variable y-coordinate of the upper-left corner of the image.
     * @param scaleX the variable x-scale of the image.
     * @param scaleY the variable y-scale of the image.
     */
    public ImageSprite(Image          image,
                       VariableInt    x,
                       VariableInt    y,
                       VariableDouble scaleX,
                       VariableDouble scaleY)
    {
        this.image  = image;
        this.x      = x;
        this.y      = y;
        this.scaleX = scaleX;
        this.scaleY = scaleY;
    }


    // Implementation for Sprite.

    public void paint(Graphics graphics, long time)
    {
        int xt = x.getInt(time);
        int yt = y.getInt(time);

        double scale_x = scaleX.getDouble(time);
        double scale_y = scaleY.getDouble(time);

        int width  = (int)(image.getWidth(null)  * scale_x);
        int height = (int)(image.getHeight(null) * scale_y);

        graphics.drawImage(image, xt, yt, width, height, null);
    }
}
