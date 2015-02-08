
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class ConstantAdder
implements   ConstantVisitor
{
    private final ConstantPoolEditor constantPoolEditor;

    private int constantIndex;


    /**
     * Creates a new ConstantAdder that will copy constants into the given
     * target class.
     */
    public ConstantAdder(ProgramClass targetClass)
    {
        constantPoolEditor = new ConstantPoolEditor(targetClass);
    }


    /**
     * Adds a copy of the specified constant in the given class and returns
     * its index. If the specified index is 0, the returned value is 0 too.
     */
    public int addConstant(Clazz clazz, int constantIndex)
    {
        clazz.constantPoolEntryAccept(constantIndex, this);

        return this.constantIndex;
    }


    /**
     * Adds a copy of the given constant in the given class and returns
     * its index.
     */
    public int addConstant(Clazz clazz, Constant constant)
    {
        constant.accept(clazz, this);

        return this.constantIndex;
    }


    /**
     * Returns the index of the most recently created constant in the constant
     * pool of the target class.
     */
    public int getConstantIndex()
    {
        return constantIndex;
    }


    // Implementations for ConstantVisitor.

    public void visitIntegerConstant(Clazz clazz, IntegerConstant integerConstant)
    {
        constantIndex =
            constantPoolEditor.addIntegerConstant(integerConstant.getValue());
    }


    public void visitLongConstant(Clazz clazz, LongConstant longConstant)
    {
        constantIndex =
            constantPoolEditor.addLongConstant(longConstant.getValue());
    }


    public void visitFloatConstant(Clazz clazz, FloatConstant floatConstant)
    {
        constantIndex =
            constantPoolEditor.addFloatConstant(floatConstant.getValue());
    }


    public void visitDoubleConstant(Clazz clazz, DoubleConstant doubleConstant)
    {
        constantIndex =
            constantPoolEditor.addDoubleConstant(doubleConstant.getValue());
    }


    public void visitStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        constantIndex =
            constantPoolEditor.addStringConstant(stringConstant.getString(clazz),
                                                 stringConstant.referencedClass,
                                                 stringConstant.referencedMember);
    }


    public void visitUtf8Constant(Clazz clazz, Utf8Constant utf8Constant)
    {
        constantIndex =
            constantPoolEditor.addUtf8Constant(utf8Constant.getString());
    }


    public void visitFieldrefConstant(Clazz clazz, FieldrefConstant fieldrefConstant)
    {
        // First add the referenced class constant, with its own referenced class.
        clazz.constantPoolEntryAccept(fieldrefConstant.u2classIndex, this);

        // Then add the actual field reference constant, with its referenced
        // class and class member.
        constantIndex =
            constantPoolEditor.addFieldrefConstant(constantIndex,
                                                   fieldrefConstant.getName(clazz),
                                                   fieldrefConstant.getType(clazz),
                                                   fieldrefConstant.referencedClass,
                                                   fieldrefConstant.referencedMember);
    }


    public void visitInterfaceMethodrefConstant(Clazz clazz, InterfaceMethodrefConstant interfaceMethodrefConstant)
    {
        // First add the referenced class constant, with its own referenced class.
        clazz.constantPoolEntryAccept(interfaceMethodrefConstant.u2classIndex, this);

        // Then add the actual interface method reference constant, with its
        // referenced class and class member.
        constantIndex =
            constantPoolEditor.addInterfaceMethodrefConstant(constantIndex,
                                                             interfaceMethodrefConstant.getName(clazz),
                                                             interfaceMethodrefConstant.getType(clazz),
                                                             interfaceMethodrefConstant.referencedClass,
                                                             interfaceMethodrefConstant.referencedMember);
    }


    public void visitMethodrefConstant(Clazz clazz, MethodrefConstant methodrefConstant)
    {
        // First add the referenced class constant, with its own referenced class.
        clazz.constantPoolEntryAccept(methodrefConstant.u2classIndex, this);

        // Then add the actual method reference constant, with its referenced
        // class and class member.
        constantIndex =
            constantPoolEditor.addMethodrefConstant(constantIndex,
                                                    methodrefConstant.getName(clazz),
                                                    methodrefConstant.getType(clazz),
                                                    methodrefConstant.referencedClass,
                                                    methodrefConstant.referencedMember);
    }


    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        // Add the class constant, with its referenced class..
        constantIndex =
            constantPoolEditor.addClassConstant(classConstant.getName(clazz),
                                                classConstant.referencedClass);
    }


    public void visitNameAndTypeConstant(Clazz clazz, NameAndTypeConstant nameAndTypeConstant)
    {
        constantIndex =
            constantPoolEditor.addNameAndTypeConstant(nameAndTypeConstant.getName(clazz),
                                                      nameAndTypeConstant.getType(clazz));
    }
}
