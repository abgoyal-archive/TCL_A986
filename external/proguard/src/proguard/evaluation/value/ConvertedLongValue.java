
package proguard.evaluation.value;

final class ConvertedLongValue extends SpecificLongValue
{
    private final Value value;


    /**
     * Creates a new converted long value of the given value.
     */
    public ConvertedLongValue(Value value)
    {
        this.value = value;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.value.equals(((ConvertedLongValue)object).value);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               value.hashCode();
    }


    public String toString()
    {
        return "(long)("+value+")";
    }
}