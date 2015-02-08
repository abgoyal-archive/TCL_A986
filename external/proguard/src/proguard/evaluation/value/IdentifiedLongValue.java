
package proguard.evaluation.value;

final class IdentifiedLongValue extends SpecificLongValue
{
    private final ValueFactory valuefactory;
    private final int          id;


    /**
     * Creates a new long value with the given ID.
     */
    public IdentifiedLongValue(ValueFactory valuefactory, int id)
    {
        this.valuefactory = valuefactory;
        this.id           = id;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.valuefactory.equals(((IdentifiedLongValue)object).valuefactory) &&
               this.id == ((IdentifiedLongValue)object).id;
    }


    public int hashCode()
    {
        return super.hashCode() ^
               valuefactory.hashCode() ^
               id;
    }


    public String toString()
    {
        return "l"+id;
    }
}