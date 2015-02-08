
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.MemberVisitor;
import proguard.evaluation.value.Value;
import proguard.optimize.evaluation.StoringInvocationUnit;

public class ConstantMemberFilter
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final MemberVisitor constantMemberVisitor;


    /**
     * Creates a new ConstantMemberFilter.
     * @param constantMemberVisitor the <code>MemberVisitor</code> to which
     *                              visits to constant members will be delegated.
     */
    public ConstantMemberFilter(MemberVisitor constantMemberVisitor)
    {
        this.constantMemberVisitor = constantMemberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        Value value = StoringInvocationUnit.getFieldValue(programField);
        if (value != null &&
            value.isParticular())
        {
            constantMemberVisitor.visitProgramField(programClass, programField);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        Value value = StoringInvocationUnit.getMethodReturnValue(programMethod);
        if (value != null &&
            value.isParticular())
        {
            constantMemberVisitor.visitProgramMethod(programClass, programMethod);
        }
    }
}
