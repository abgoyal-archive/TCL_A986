
package proguard.evaluation.value;

final class ParticularDoubleValue extends SpecificDoubleValue
{
    private final double value;


    /**
     * Creates a new particular double value.
     */
    public ParticularDoubleValue(double value)
    {
        this.value = value;
    }


    // Implementations for DoubleValue.

    public double value()
    {
        return value;
    }


    // Implementations of unary methods of DoubleValue.

    public DoubleValue negate()
    {
        return new ParticularDoubleValue(-value);
    }

    public IntegerValue convertToInteger()
    {
        return new ParticularIntegerValue((int)value);
    }

    public LongValue convertToLong()
    {
        return new ParticularLongValue((long)value);
    }

    public FloatValue convertToFloat()
    {
        return new ParticularFloatValue((float)value);
    }


    // Implementations of binary methods of DoubleValue.

    public DoubleValue generalize(DoubleValue other)
    {
        return other.generalize(this);
    }

    public DoubleValue add(DoubleValue other)
    {
        return value == 0.0 ? other : other.add(this);
    }

    public DoubleValue subtract(DoubleValue other)
    {
        return value == 0.0 ? other.negate() : other.subtractFrom(this);
    }

    public DoubleValue subtractFrom(DoubleValue other)
    {
        return value == 0.0 ? other : other.subtract(this);
    }

    public DoubleValue multiply(DoubleValue other)
    {
        return other.multiply(this);
    }

    public DoubleValue divide(DoubleValue other)
    {
        return other.divideOf(this);
    }

    public DoubleValue divideOf(DoubleValue other)
    {
        return other.divide(this);
    }

    public DoubleValue remainder(DoubleValue other)
    {
        return other.remainderOf(this);
    }

    public DoubleValue remainderOf(DoubleValue other)
    {
        return other.remainder(this);
    }

    public IntegerValue compare(DoubleValue other)
    {
        return other.compareReverse(this);
    }


    // Implementations of binary DoubleValue methods with ParticularDoubleValue
    // arguments.

    public DoubleValue generalize(ParticularDoubleValue other)
    {
        return this.value == other.value ? this : ValueFactory.DOUBLE_VALUE;
    }

    public DoubleValue add(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(this.value + other.value);
    }

    public DoubleValue subtract(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(this.value - other.value);
    }

    public DoubleValue subtractFrom(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(other.value - this.value);
    }

    public DoubleValue multiply(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(this.value * other.value);
    }

    public DoubleValue divide(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(this.value / other.value);
    }

    public DoubleValue divideOf(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(other.value / this.value);
    }

    public DoubleValue remainder(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(this.value % other.value);
    }

    public DoubleValue remainderOf(ParticularDoubleValue other)
    {
        return new ParticularDoubleValue(other.value % this.value);
    }

    public IntegerValue compare(ParticularDoubleValue other)
    {
        return this.value <  other.value ? SpecificValueFactory.INTEGER_VALUE_M1 :
               this.value == other.value ? SpecificValueFactory.INTEGER_VALUE_0  :
                                           SpecificValueFactory.INTEGER_VALUE_1;
    }


    // Implementations for Value.

    public boolean isParticular()
    {
        return true;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        return super.equals(object) &&
               this.value == ((ParticularDoubleValue)object).value;
    }


    public int hashCode()
    {
        return super.hashCode() ^
               (int)Double.doubleToLongBits(value);
    }


    public String toString()
    {
        return value+"d";
    }
}