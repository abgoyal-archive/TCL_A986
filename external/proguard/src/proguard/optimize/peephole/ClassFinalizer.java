
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;
import proguard.optimize.KeepMarker;

public class ClassFinalizer
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private final ClassVisitor extraClassVisitor;


    /**
     * Creates a new ClassFinalizer.
     */
    public ClassFinalizer()
    {
        this(null);
    }


    /**
     * Creates a new ClassFinalizer.
     * @param extraClassVisitor an optional extra visitor for all finalized
     *                          classes.
     */
    public ClassFinalizer(ClassVisitor  extraClassVisitor)
    {
        this.extraClassVisitor = extraClassVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // If the class is not final/interface/abstract,
        // and it is not being kept,
        // and it doesn't have any subclasses,
        // then make it final.
        if ((programClass.u2accessFlags & (ClassConstants.INTERNAL_ACC_FINAL     |
                                           ClassConstants.INTERNAL_ACC_INTERFACE |
                                           ClassConstants.INTERNAL_ACC_ABSTRACT)) == 0 &&
            !KeepMarker.isKept(programClass)                                           &&
            programClass.subClasses == null)
        {
            programClass.u2accessFlags |= ClassConstants.INTERNAL_ACC_FINAL;

            // Visit the class, if required.
            if (extraClassVisitor != null)
            {
                extraClassVisitor.visitProgramClass(programClass);
            }
        }
    }
}
