
package proguard.evaluation.value;

final class ConvertedIntegerValue extends SpecificIntegerValue
{
    private final Value value;


    /**
     * Creates a new converted integer value of the given value.
     */
    public ConvertedIntegerValue(Value value)
    {
        this.value = value;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.value.equals(((ConvertedIntegerValue)object).value);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               value.hashCode();
    }


    public String toString()
    {
        return "(int)("+value+")";
    }
}