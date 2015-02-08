
package proguard.classfile.visitor;

import proguard.classfile.*;


public class LibraryMemberFilter implements MemberVisitor
{
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new ProgramMemberFilter.
     * @param memberVisitor     the <code>MemberVisitor</code> to which
     *                          visits will be delegated.
     */
    public LibraryMemberFilter(MemberVisitor memberVisitor)
    {
        this.memberVisitor = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        // Don't delegate visits to program members.
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Don't delegate visits to program members.
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        memberVisitor.visitLibraryField(libraryClass, libraryField);
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        memberVisitor.visitLibraryMethod(libraryClass, libraryMethod);
    }
}
