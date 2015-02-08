
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.ExceptionInfoVisitor;

public class ExceptionHandlerFilter
implements   ExceptionInfoVisitor
{
    private final int                  startOffset;
    private final int                  endOffset;
    private final ExceptionInfoVisitor exceptionInfoVisitor;


    /**
     * Creates a new ExceptionHandlerFilter.
     * @param startOffset          the start of the instruction offset range.
     * @param endOffset            the end of the instruction offset range.
     * @param exceptionInfoVisitor the ExceptionInfoVisitor to which visits
     *                             will be delegated.
     */
    public ExceptionHandlerFilter(int                  startOffset,
                                  int                  endOffset,
                                  ExceptionInfoVisitor exceptionInfoVisitor)
    {
        this.startOffset          = startOffset;
        this.endOffset            = endOffset;
        this.exceptionInfoVisitor = exceptionInfoVisitor;
    }


    // Implementations for ExceptionInfoVisitor.

    public void visitExceptionInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, ExceptionInfo exceptionInfo)
    {
        int handlerPC = exceptionInfo.u2handlerPC;
        if (handlerPC >= startOffset &&
            handlerPC <  endOffset)
        {
            exceptionInfoVisitor.visitExceptionInfo(clazz, method, codeAttribute, exceptionInfo);
        }
    }
}