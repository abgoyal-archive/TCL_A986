
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class DoubleConstant extends Constant
{
    public double f8value;


    /**
     * Creates an uninitialized DoubleConstant.
     */
    public DoubleConstant()
    {
    }


    /**
     * Creates a new DoubleConstant with the given double value.
     */
    public DoubleConstant(double value)
    {
        f8value = value;
    }


    /**
     * Returns the double value of this DoubleConstant.
     */
    public double getValue()
    {
        return f8value;
    }


    /**
     * Sets the double value of this DoubleConstant.
     */
    public void setValue(double value)
    {
        f8value = value;
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_Double;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitDoubleConstant(clazz, this);
    }
}
