
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class IntegerConstant extends Constant
{
    public int u4value;


    /**
     * Creates an uninitialized IntegerConstant.
     */
    public IntegerConstant()
    {
    }


    /**
     * Creates a new IntegerConstant with the given integer value.
     */
    public IntegerConstant(int value)
    {
        u4value = value;
    }


    /**
     * Returns the integer value of this IntegerConstant.
     */
    public int getValue()
    {
        return u4value;
    }


    /**
     * Sets the integer value of this IntegerConstant.
     */
    public void setValue(int value)
    {
        u4value = value;
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_Integer;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitIntegerConstant(clazz, this);
    }
}
