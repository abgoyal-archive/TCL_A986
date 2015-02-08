
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.ExceptionInfoVisitor;

public class ExceptionCounter implements ExceptionInfoVisitor
{
    private int count;


    /**
     * Returns the number of exceptions that has been visited so far.
     */
    public int getCount()
    {
        return count;
    }


    // Implementations for ExceptionInfoVisitor.

    public void visitExceptionInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, ExceptionInfo exceptionInfo)
    {
        count++;
    }
}
