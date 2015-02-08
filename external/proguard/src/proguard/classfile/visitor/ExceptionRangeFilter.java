
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.ExceptionInfoVisitor;

public class ExceptionRangeFilter
implements   ExceptionInfoVisitor
{
    private final int                  startOffset;
    private final int                  endOffset;
    private final ExceptionInfoVisitor exceptionInfoVisitor;


    /**
     * Creates a new ExceptionRangeFilter.
     * @param startOffset          the start offset of the instruction range.
     * @param endOffset            the end offset of the instruction range.
     * @param exceptionInfoVisitor the ExceptionInfoVisitor to which visits
     *                             will be delegated.
     */
    public ExceptionRangeFilter(int                  startOffset,
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
        if (exceptionInfo.isApplicable(startOffset, endOffset))
        {
            exceptionInfoVisitor.visitExceptionInfo(clazz, method, codeAttribute, exceptionInfo);
        }
    }
}
