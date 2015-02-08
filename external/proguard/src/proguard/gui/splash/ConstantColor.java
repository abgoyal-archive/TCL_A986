
package proguard.gui.splash;

import java.awt.*;

public class ConstantColor implements VariableColor
{
    private final Color value;


    /**
     * Creates a new ConstantColor.
     * @param value the constant value.
     */
    public ConstantColor(Color value)
    {
        this.value = value;
    }


    // Implementation for VariableColor.

    public Color getColor(long time)
    {
        return value;
    }
}
