
package proguard.classfile.visitor;

import proguard.classfile.*;


public class ProgramMemberFilter implements MemberVisitor
{
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new ProgramMemberFilter.
     * @param memberVisitor     the <code>MemberVisitor</code> to which
     *                          visits will be delegated.
     */
    public ProgramMemberFilter(MemberVisitor memberVisitor)
    {
        this.memberVisitor = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        memberVisitor.visitProgramField(programClass, programField);
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        memberVisitor.visitProgramMethod(programClass, programMethod);
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        // Don't delegate visits to library members.
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        // Don't delegate visits to library members.
    }
}
