
package proguard.gui.splash;

public class ConstantInt implements VariableInt
{
    private final int value;


    /**
     * Creates a new ConstantInt.
     * @param value the constant value.
     */
    public ConstantInt(int value)
    {
        this.value = value;
    }


    // Implementation for VariableInt.

    public int getInt(long time)
    {
        return value;
    }
}
