
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class FloatConstant extends Constant
{
    public float f4value;


    /**
     * Creates an uninitialized FloatConstant.
     */
    public FloatConstant()
    {
    }


    /**
     * Creates a new FloatConstant with the given float value.
     */
    public FloatConstant(float value)
    {
        f4value = value;
    }


    /**
     * Returns the float value of this FloatConstant.
     */
    public float getValue()
    {
        return f4value;
    }


    /**
     * Sets the float value of this FloatConstant.
     */
    public void setValue(float value)
    {
        f4value = value;
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_Float;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitFloatConstant(clazz, this);
    }
}
