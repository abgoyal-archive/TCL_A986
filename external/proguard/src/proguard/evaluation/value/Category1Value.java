
package proguard.evaluation.value;

public abstract class Category1Value extends Value
{
    // Implementations for Value.

    public final Category1Value category1Value()
    {
        return this;
    }

    public final boolean isCategory2()
    {
        return false;
    }
}
