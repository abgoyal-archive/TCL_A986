
package proguard.evaluation.value;

final class ConvertedCharacterValue extends SpecificIntegerValue
{
    private final IntegerValue value;


    /**
     * Creates a new converted character value of the given integer value.
     */
    public ConvertedCharacterValue(IntegerValue value)
    {
        this.value = value;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.value.equals(((ConvertedCharacterValue)object).value);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               value.hashCode();
    }


    public String toString()
    {
        return "(char)("+value+")";
    }
}