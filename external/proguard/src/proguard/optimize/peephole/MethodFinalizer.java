
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;
import proguard.optimize.KeepMarker;

public class MethodFinalizer
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final MemberVisitor extraMemberVisitor;

    private final MemberFinder memberFinder = new MemberFinder();


    /**
     * Creates a new ClassFinalizer.
     */
    public MethodFinalizer()
    {
        this(null);
    }


    /**
     * Creates a new ClassFinalizer.
     * @param extraMemberVisitor an optional extra visitor for all finalized
     *                           methods.
     */
    public MethodFinalizer(MemberVisitor extraMemberVisitor)
    {
        this.extraMemberVisitor = extraMemberVisitor;
    }


    // Implementations for MemberVisitor.
    
    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        String name = programMethod.getName(programClass);

        // If the method is not already private/static/final/abstract,
        // and it is not a constructor,
        // and its class is final,
        //     or it is not being kept and it is not overridden,
        // then make it final.
        if ((programMethod.u2accessFlags & (ClassConstants.INTERNAL_ACC_PRIVATE |
                                            ClassConstants.INTERNAL_ACC_STATIC  |
                                            ClassConstants.INTERNAL_ACC_FINAL   |
                                            ClassConstants.INTERNAL_ACC_ABSTRACT)) == 0 &&
            !name.equals(ClassConstants.INTERNAL_METHOD_NAME_INIT)                      &&
            ((programClass.u2accessFlags & ClassConstants.INTERNAL_ACC_FINAL) != 0 ||
             (!KeepMarker.isKept(programMethod) &&
              (programClass.subClasses == null ||
               !memberFinder.isOverriden(programClass, programMethod)))))
        {
            programMethod.u2accessFlags |= ClassConstants.INTERNAL_ACC_FINAL;

            // Visit the method, if required.
            if (extraMemberVisitor != null)
            {
                extraMemberVisitor.visitProgramMethod(programClass, programMethod);
            }
        }
    }
}