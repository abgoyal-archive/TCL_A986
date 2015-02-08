
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;
import proguard.evaluation.value.Value;
import proguard.optimize.evaluation.StoringInvocationUnit;
import proguard.optimize.info.ParameterUsageMarker;

public class ConstantParameterFilter
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final MemberVisitor constantParameterVisitor;


    /**
     * Creates a new ConstantParameterFilter.
     * @param constantParameterVisitor the <code>MemberVisitor</code> to which
     *                                 visits will be delegated.
     */
    public ConstantParameterFilter(MemberVisitor constantParameterVisitor)
    {
        this.constantParameterVisitor = constantParameterVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // All parameters of non-static methods are shifted by one in the local
        // variable frame.
        int firstParameterIndex =
            (programMethod.getAccessFlags() & ClassConstants.INTERNAL_ACC_STATIC) != 0 ?
                0 : 1;

        int parameterCount =
            ClassUtil.internalMethodParameterCount(programMethod.getDescriptor(programClass));

        for (int index = firstParameterIndex; index < parameterCount; index++)
        {
            Value value = StoringInvocationUnit.getMethodParameterValue(programMethod, index);
            if (value != null &&
                value.isParticular())
            {
                constantParameterVisitor.visitProgramMethod(programClass, programMethod);
            }
        }
    }
}