
package proguard.evaluation.value;

final class ConvertedShortValue extends SpecificIntegerValue
{
    private final IntegerValue value;


    /**
     * Creates a new converted short value of the given integer value.
     */
    public ConvertedShortValue(IntegerValue value)
    {
        this.value = value;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.value.equals(((ConvertedShortValue)object).value);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               value.hashCode();
    }


    public String toString()
    {
        return "(short)("+value+")";
    }
}