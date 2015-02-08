
package proguard.evaluation.value;

final class ComparisonValue extends SpecificIntegerValue
{
    private final Value value1;
    private final Value value2;


    /**
     * Creates a new comparison integer value of the two given scalar values.
     */
    public ComparisonValue(Value integerValue1,
                           Value integerValue2)
    {
        this.value1 = integerValue1;
        this.value2 = integerValue2;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.value1.equals(((ComparisonValue)object).value1) &&
               this.value2.equals(((ComparisonValue)object).value2);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               value1.hashCode() ^
               value2.hashCode();
    }


    public String toString()
    {
        return "("+value1+"~"+ value2 +")";
    }
}