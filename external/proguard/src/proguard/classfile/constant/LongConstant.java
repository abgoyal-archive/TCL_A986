
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class LongConstant extends Constant
{
    public long u8value;


    /**
     * Creates an uninitialized LongConstant.
     */
    public LongConstant()
    {
    }


    /**
     * Creates a new LongConstant with the given long value.
     */
    public LongConstant(long value)
    {
        u8value = value;
    }


    /**
     * Returns the long value of this LongConstant.
     */
    public long getValue()
    {
        return u8value;
    }


    /**
     * Sets the long value of this LongConstant.
     */
    public void setValue(long value)
    {
        u8value = value;
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_Long;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitLongConstant(clazz, this);
    }
}
