
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.editor.MethodInvocationFixer;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;
import proguard.optimize.info.NonPrivateMemberMarker;

public class MemberPrivatizer
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final MemberVisitor extraMemberVisitor;


    /**
     * Creates a new MemberPrivatizer.
     */
    public MemberPrivatizer()
    {
        this(null);
    }


    /**
     * Creates a new MemberPrivatizer.
     * @param extraMemberVisitor an optional extra visitor for all privatized
     *                           class members.
     */
    public MemberPrivatizer(MemberVisitor extraMemberVisitor)
    {
        this.extraMemberVisitor = extraMemberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        // Is the field unmarked?
        if (NonPrivateMemberMarker.canBeMadePrivate(programField))
        {
            // Make the field private.
            programField.u2accessFlags =
                AccessUtil.replaceAccessFlags(programField.u2accessFlags,
                                              ClassConstants.INTERNAL_ACC_PRIVATE);

            // Visit the field, if required.
            if (extraMemberVisitor != null)
            {
                extraMemberVisitor.visitProgramField(programClass, programField);
            }
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Is the method unmarked?
        if (NonPrivateMemberMarker.canBeMadePrivate(programMethod))
        {
            // Make the method private.
            programMethod.u2accessFlags =
                AccessUtil.replaceAccessFlags(programMethod.u2accessFlags,
                                              ClassConstants.INTERNAL_ACC_PRIVATE);

            // Visit the method, if required.
            if (extraMemberVisitor != null)
            {
                extraMemberVisitor.visitProgramMethod(programClass, programMethod);
            }
        }
    }
}
