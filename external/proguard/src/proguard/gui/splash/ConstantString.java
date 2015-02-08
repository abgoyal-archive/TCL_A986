
package proguard.gui.splash;

public class ConstantString implements VariableString
{
    private final String value;


    /**
     * Creates a new ConstantString.
     * @param value the constant value.
     */
    public ConstantString(String value)
    {
        this.value = value;
    }


    // Implementation for VariableString.

    public String getString(long time)
    {
        return value;
    }
}
