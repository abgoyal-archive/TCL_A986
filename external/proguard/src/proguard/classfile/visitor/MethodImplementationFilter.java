
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;

public class MethodImplementationFilter
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new MethodImplementationFilter.
     * @param memberVisitor     the <code>MemberVisitor</code> to which
     *                          visits will be delegated.
     */
    public MethodImplementationFilter(MemberVisitor memberVisitor)
    {
        this.memberVisitor = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (programClass.mayHaveImplementations(programMethod))
        {
            memberVisitor.visitProgramMethod(programClass, programMethod);
        }
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        if (libraryClass.mayHaveImplementations(libraryMethod))
        {
            memberVisitor.visitLibraryMethod(libraryClass, libraryMethod);
        }
    }
}
