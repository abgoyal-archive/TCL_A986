
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.ExceptionInfoVisitor;

public class ExceptionExcludedOffsetFilter
implements   ExceptionInfoVisitor
{
    private final int                  instructionOffset;
    private final ExceptionInfoVisitor exceptionInfoVisitor;


    /**
     * Creates a new ExceptionExcludedOffsetFilter.
     * @param instructionOffset    the instruction offset.
     * @param exceptionInfoVisitor the ExceptionInfoVisitor to which visits
     *                             will be delegated.
     */
    public ExceptionExcludedOffsetFilter(int                  instructionOffset,
                                         ExceptionInfoVisitor exceptionInfoVisitor)
    {
        this.instructionOffset    = instructionOffset;
        this.exceptionInfoVisitor = exceptionInfoVisitor;
    }


    // Implementations for ExceptionInfoVisitor.

    public void visitExceptionInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, ExceptionInfo exceptionInfo)
    {
        if (!exceptionInfo.isApplicable(instructionOffset))
        {
            exceptionInfoVisitor.visitExceptionInfo(clazz, method, codeAttribute, exceptionInfo);
        }
    }
}
