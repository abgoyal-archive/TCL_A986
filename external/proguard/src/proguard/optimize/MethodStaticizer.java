
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.editor.MethodInvocationFixer;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.MemberVisitor;
import proguard.optimize.info.ParameterUsageMarker;
import proguard.optimize.peephole.VariableShrinker;

public class MethodStaticizer
extends      SimplifiedVisitor
implements   MemberVisitor,
             AttributeVisitor
{
    private final MemberVisitor extraStaticMemberVisitor;


    /**
     * Creates a new MethodStaticizer.
     */
    public MethodStaticizer()
    {
        this(null);
    }


    /**
     * Creates a new MethodStaticizer with an extra visitor.
     * @param extraStaticMemberVisitor an optional extra visitor for all
     *                                 methods that have been made static.
     */
    public MethodStaticizer(MemberVisitor extraStaticMemberVisitor)
    {
        this.extraStaticMemberVisitor = extraStaticMemberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Is the 'this' parameter being used?
        if (!ParameterUsageMarker.isParameterUsed(programMethod, 0))
        {
            // Make the method static.
            programMethod.u2accessFlags =
                (programMethod.getAccessFlags() & ~ClassConstants.INTERNAL_ACC_FINAL) |
                ClassConstants.INTERNAL_ACC_STATIC;

            // Visit the method, if required.
            if (extraStaticMemberVisitor != null)
            {
                extraStaticMemberVisitor.visitProgramMethod(programClass, programMethod);
            }
        }
    }
}
