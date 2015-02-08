
package proguard.gui.splash;

import java.awt.*;

public class ConstantFont implements VariableFont
{
    private final Font value;

    public ConstantFont(Font value)
    {
        this.value = value;
    }


    // Implementation for VariableFont.

    public Font getFont(long time)
    {
        return value;
    }
}
