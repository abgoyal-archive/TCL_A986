
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.ExceptionInfoVisitor;

public class ExceptionInfoAdder
implements   ExceptionInfoVisitor
{
    private final ConstantAdder         constantAdder;
    private final CodeAttributeComposer codeAttributeComposer;


    /**
     * Creates a new ExceptionAdder that will copy exceptions into the given
     * target code attribute.
     */
    public ExceptionInfoAdder(ProgramClass          targetClass,
                              CodeAttributeComposer targetComposer)
    {
        constantAdder         = new ConstantAdder(targetClass);
        codeAttributeComposer = targetComposer;
    }


    // Implementations for ExceptionInfoVisitor.

    public void visitExceptionInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, ExceptionInfo exceptionInfo)
    {
        // Create a copy of the exception info.
        ExceptionInfo newExceptionInfo =
            new ExceptionInfo(exceptionInfo.u2startPC,
                              exceptionInfo.u2endPC,
                              exceptionInfo.u2handlerPC,
                              exceptionInfo.u2catchType == 0 ? 0 :
                                  constantAdder.addConstant(clazz, exceptionInfo.u2catchType));

        // Add the completed exception info.
        codeAttributeComposer.appendException(newExceptionInfo);
    }
}