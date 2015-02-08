
package proguard.evaluation;

import proguard.evaluation.value.Value;

public class TracedVariables extends Variables
{
    public static final int NONE = -1;


    private Value     producerValue;
    private Variables producerVariables;
    private int       initializationIndex;


    /**
     * Creates a new TracedVariables with a given size.
     */
    public TracedVariables(int size)
    {
        super(size);

        producerVariables = new Variables(size);
    }


    /**
     * Creates a new TracedVariables that is a copy of the given TracedVariables.
     */
    public TracedVariables(TracedVariables tracedVariables)
    {
        super(tracedVariables);

        producerVariables = new Variables(tracedVariables.producerVariables);
    }


    /**
     * Sets the Value that will be stored along with all store instructions.
     */
    public void setProducerValue(Value producerValue)
    {
        this.producerValue = producerValue;
    }


    /**
     * Resets the initialization index.
     */
    public void resetInitialization()
    {
        initializationIndex = NONE;
    }


    /**
     * Returns the most recent initialization index since it has been reset.
     */
    public int getInitializationIndex()
    {
        return initializationIndex;
    }


    /**
     * Gets the producer Value for the specified variable, without disturbing it.
     * @param index the variable index.
     * @return the producer value of the given variable.
     */
    public Value getProducerValue(int index)
    {
        return producerVariables.getValue(index);
    }


    /**
     * Sets the given producer Value for the specified variable, without
     * disturbing it.
     * @param index the variable index.
     * @param value the producer value to set.
     */
    public void setProducerValue(int index, Value value)
    {
        producerVariables.store(index, value);
    }


    // Implementations for Variables.

    public void reset(int size)
    {
        super.reset(size);

        producerVariables.reset(size);
    }

    public void initialize(TracedVariables other)
    {
        super.initialize(other);

        producerVariables.initialize(other.producerVariables);
    }

    public boolean generalize(TracedVariables other,
                              boolean         clearConflictingOtherVariables)
    {
        boolean variablesChanged = super.generalize(other, clearConflictingOtherVariables);
        boolean producersChanged = producerVariables.generalize(other.producerVariables, clearConflictingOtherVariables);
        /* consumerVariables.generalize(other.consumerVariables)*/

        // Clear any traces if a variable has become null.
        if (variablesChanged)
        {
            for (int index = 0; index < size; index++)
            {
                if (values[index] == null)
                {
                    producerVariables.values[index] = null;

                    if (clearConflictingOtherVariables)
                    {
                        other.producerVariables.values[index] = null;
                    }
                }
            }
        }

        return variablesChanged || producersChanged;
    }


    public void store(int index, Value value)
    {
        // Is this store operation an initialization of the variable?
        Value previousValue = super.load(index);
        if (previousValue == null ||
            previousValue.computationalType() != value.computationalType())
        {
            initializationIndex = index;
        }

        // Store the value itself in the variable.
        super.store(index, value);

        // Store the producer value in its producer variable.
        producerVariables.store(index, producerValue);

        // Account for the extra space required by Category 2 values.
        if (value.isCategory2())
        {
            producerVariables.store(index+1, producerValue);
        }
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        if (object == null ||
            this.getClass() != object.getClass())
        {
            return false;
        }

        TracedVariables other = (TracedVariables)object;

        return super.equals(object) &&
               this.producerVariables.equals(other.producerVariables);
    }


    public int hashCode()
    {
        return super.hashCode() ^
               producerVariables.hashCode();
    }


    public String toString()
    {
        StringBuffer buffer = new StringBuffer();

        for (int index = 0; index < this.size(); index++)
        {
            Value value         = this.values[index];
            Value producerValue = producerVariables.getValue(index);
            buffer = buffer.append('[')
                           .append(producerValue == null ? "empty:" : producerValue.toString())
                           .append(value         == null ? "empty"  : value.toString())
                           .append(']');
        }

        return buffer.toString();
    }
}
