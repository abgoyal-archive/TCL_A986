
package proguard.evaluation.value;

final class IdentifiedDoubleValue extends SpecificDoubleValue
{
    private final ValueFactory valuefactory;
    private final int          id;


    /**
     * Creates a new double value with the given ID.
     */
    public IdentifiedDoubleValue(ValueFactory valuefactory, int id)
    {
        this.valuefactory = valuefactory;
        this.id           = id;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.valuefactory.equals(((IdentifiedDoubleValue)object).valuefactory) &&
               this.id == ((IdentifiedDoubleValue)object).id;
    }


    public int hashCode()
    {
        return super.hashCode() ^
               valuefactory.hashCode() ^
               id;
    }


    public String toString()
    {
        return "d"+id;
    }
}