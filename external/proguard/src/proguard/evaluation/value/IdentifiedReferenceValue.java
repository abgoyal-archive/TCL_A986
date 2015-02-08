
package proguard.evaluation.value;

import proguard.classfile.Clazz;

final class IdentifiedReferenceValue extends ReferenceValue
{
    private final ValueFactory valuefactory;
    private final int          id;


    /**
     * Creates a new long value with the given ID.
     */
    public IdentifiedReferenceValue(String       type,
                                    Clazz        referencedClass,
                                    boolean      mayBeNull,
                                    ValueFactory valuefactory,
                                    int          id)
    {
        super(type, referencedClass, mayBeNull);

        this.valuefactory = valuefactory;
        this.id           = id;
    }


    // Implementations for ReferenceValue.

    public int equal(ReferenceValue other)
    {
        return this.equals(other) ? ALWAYS : MAYBE;
    }


    // Implementations for Value.

    public boolean isSpecific()
    {
        return true;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return this == object ||
               super.equals(object) &&
               this.valuefactory.equals(((IdentifiedReferenceValue)object).valuefactory) &&
               this.id == ((IdentifiedReferenceValue)object).id;
    }


    public int hashCode()
    {
        return super.hashCode() ^
               valuefactory.hashCode() ^
               id;
    }


    public String toString()
    {
        return super.toString()+'#'+id;
    }
}