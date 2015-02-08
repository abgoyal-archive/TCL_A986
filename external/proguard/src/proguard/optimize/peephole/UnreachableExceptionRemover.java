
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.*;
import proguard.classfile.instruction.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.optimize.info.ExceptionInstructionChecker;

public class UnreachableExceptionRemover
extends      SimplifiedVisitor
implements   AttributeVisitor,
             ExceptionInfoVisitor
{
    private final ExceptionInfoVisitor extraExceptionInfoVisitor;


    private final ExceptionInstructionChecker exceptionInstructionChecker = new ExceptionInstructionChecker();


    /**
     * Creates a new UnreachableExceptionRemover.
     */
    public UnreachableExceptionRemover()
    {
        this(null);
    }


    /**
     * Creates a new UnreachableExceptionRemover.
     * @param extraExceptionInfoVisitor an optional extra visitor for all
     *                                  removed exceptions.
     */
    public UnreachableExceptionRemover(ExceptionInfoVisitor extraExceptionInfoVisitor)
    {
        this.extraExceptionInfoVisitor = extraExceptionInfoVisitor;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Go over the exception table.
        codeAttribute.exceptionsAccept(clazz, method, this);

        // Remove exceptions with empty code blocks.
        codeAttribute.u2exceptionTableLength =
            removeEmptyExceptions(codeAttribute.exceptionTable,
                                  codeAttribute.u2exceptionTableLength);
    }


    // Implementations for ExceptionInfoVisitor.

    public void visitExceptionInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, ExceptionInfo exceptionInfo)
    {
        if (!mayThrowExceptions(clazz,
                                method,
                                codeAttribute,
                                exceptionInfo.u2startPC,
                                exceptionInfo.u2endPC))
        {
            // Make the code block empty.
            exceptionInfo.u2endPC = exceptionInfo.u2startPC;

            if (extraExceptionInfoVisitor != null)
            {
                extraExceptionInfoVisitor.visitExceptionInfo(clazz, method, codeAttribute, exceptionInfo);
            }
        }
    }


    // Small utility methods.

    /**
     * Returns whether the specified block of code may throw exceptions.
     */
    private boolean mayThrowExceptions(Clazz         clazz,
                                       Method        method,
                                       CodeAttribute codeAttribute,
                                       int           startOffset,
                                       int           endOffset)
    {
        byte[] code = codeAttribute.code;

        // Go over all instructions.
        int offset = startOffset;
        while (offset < endOffset)
        {
            // Get the current instruction.
            Instruction instruction = InstructionFactory.create(code, offset);

            // Check if it may be throwing exceptions.
            if (exceptionInstructionChecker.mayThrowExceptions(clazz,
                                                               method,
                                                               codeAttribute,
                                                               offset,
                                                               instruction))
            {
                return true;
            }

            // Go to the next instruction.
            offset += instruction.length(offset);
        }

        return false;
    }


    /**
     * Returns the given list of exceptions, without the ones that have empty
     * code blocks.
     */
    private int removeEmptyExceptions(ExceptionInfo[] exceptionInfos,
                                      int             exceptionInfoCount)
    {
        // Overwrite all empty exceptions.
        int newIndex = 0;
        for (int index = 0; index < exceptionInfoCount; index++)
        {
            ExceptionInfo exceptionInfo = exceptionInfos[index];
            if (exceptionInfo.u2startPC < exceptionInfo.u2endPC)
            {
                exceptionInfos[newIndex++] = exceptionInfo;
            }
        }

        return newIndex;
    }
}
