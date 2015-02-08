
package proguard.classfile.constant;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class InterfaceMethodrefConstant extends RefConstant
{
    /**
     * Creates an uninitialized InterfaceMethodrefConstant.
     */
    public InterfaceMethodrefConstant()
    {
    }


    /**
     * Creates a new InterfaceMethodrefConstant with the given name and type indices.
     * @param u2classIndex         the index   of the class in the constant pool.
     * @param u2nameAndTypeIndex   the index   of the name and type entry in the constant pool.
     * @param referencedClass      the referenced class.
     * @param referencedMember     the referenced member info.
     */
    public InterfaceMethodrefConstant(int    u2classIndex,
                                      int    u2nameAndTypeIndex,
                                      Clazz  referencedClass,
                                      Member referencedMember)
    {
        this.u2classIndex       = u2classIndex;
        this.u2nameAndTypeIndex = u2nameAndTypeIndex;
        this.referencedClass    = referencedClass;
        this.referencedMember   = referencedMember;
    }


    // Implementations for Constant.

    public int getTag()
    {
        return ClassConstants.CONSTANT_InterfaceMethodref;
    }

    public void accept(Clazz clazz, ConstantVisitor constantVisitor)
    {
        constantVisitor.visitInterfaceMethodrefConstant(clazz, this);
    }
}
