
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.ExceptionsAttribute;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class ExceptionAdder
extends      SimplifiedVisitor
implements   ConstantVisitor
{
    private final ConstantAdder             constantAdder;
    private final ExceptionsAttributeEditor exceptionsAttributeEditor;


    /**
     * Creates a new ExceptionAdder that will copy classes into the given
     * target exceptions attribute.
     */
    public ExceptionAdder(ProgramClass        targetClass,
                          ExceptionsAttribute targetExceptionsAttribute)
    {
        constantAdder             = new ConstantAdder(targetClass);
        exceptionsAttributeEditor = new ExceptionsAttributeEditor(targetExceptionsAttribute);
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        // Add a class constant to the constant pool.
        constantAdder.visitClassConstant(clazz, classConstant);

        // Add the index of the class constant to the list of exceptions.
        exceptionsAttributeEditor.addException(constantAdder.getConstantIndex());
    }
}
