
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.visitor.MemberVisitor;

public class MemberSpecialNameFilter implements MemberVisitor
{
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new MemberSpecialNameFilter.
     * @param memberVisitor the <code>MemberVisitor</code> to which
     *                      visits will be delegated.
     */
    public MemberSpecialNameFilter(MemberVisitor memberVisitor)
    {
        this.memberVisitor = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        if (isSpecialName(programField))
        {
            memberVisitor.visitProgramField(programClass, programField);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (isSpecialName(programMethod))
        {
            memberVisitor.visitProgramMethod(programClass, programMethod);
        }
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        if (isSpecialName(libraryField))
        {
            memberVisitor.visitLibraryField(libraryClass, libraryField);
        }
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        if (isSpecialName(libraryMethod))
        {
            memberVisitor.visitLibraryMethod(libraryClass, libraryMethod);
        }
    }


    // Small utility methods.

    /**
     * Returns whether the given class member has a special new name.
     * @param member the class member.
     */
    private static boolean isSpecialName(Member member)
    {
        return SpecialNameFactory.isSpecialName(MemberObfuscator.newMemberName(member));
    }
}
