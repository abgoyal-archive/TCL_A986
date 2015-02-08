
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.MemberVisitor;

public class NoSideEffectMethodMarker
extends      SimplifiedVisitor
implements   MemberVisitor
{
    // A visitor info flag to indicate the visitor accepter is being kept,
    // but that it doesn't have any side effects.
    private static final Object KEPT_BUT_NO_SIDE_EFFECTS = new Object();


    // Implementations for MemberVisitor.

    public void visitAnyMember(Clazz Clazz, Member member)
    {
        // Ignore any attempts to mark fields.
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        markNoSideEffects(programMethod);
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        markNoSideEffects(libraryMethod);
    }


    // Small utility methods.

    private static void markNoSideEffects(Method method)
    {
        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        if (info != null)
        {
            info.setNoSideEffects();
        }
        else
        {
            MethodLinker.lastMember(method).setVisitorInfo(KEPT_BUT_NO_SIDE_EFFECTS);
        }
    }


    public static boolean hasNoSideEffects(Method method)
    {
        if (MethodLinker.lastVisitorAccepter(method).getVisitorInfo() == KEPT_BUT_NO_SIDE_EFFECTS)
        {
            return true;
        }

        MethodOptimizationInfo info = MethodOptimizationInfo.getMethodOptimizationInfo(method);
        return info != null &&
               info.hasNoSideEffects();
    }
}
