
package proguard.gui.splash;

public class ConstantDouble implements VariableDouble
{
    private final double value;


    /**
     * Creates a new ConstantDouble.
     * @param value the constant value.
     */
    public ConstantDouble(double value)
    {
        this.value = value;
    }


    // Implementation for VariableDouble.

    public double getDouble(long time)
    {
        return value;
    }
}
