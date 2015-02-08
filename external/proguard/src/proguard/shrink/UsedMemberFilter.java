
package proguard.shrink;

import proguard.classfile.*;
import proguard.classfile.visitor.MemberVisitor;

public class UsedMemberFilter
implements   MemberVisitor
{
    private final UsageMarker   usageMarker;
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new UsedClassFilter.
     * @param usageMarker   the usage marker that is used to mark the classes
     *                      and class members.
     * @param memberVisitor the member visitor to which the visiting will be
     *                      delegated.
     */
    public UsedMemberFilter(UsageMarker   usageMarker,
                            MemberVisitor memberVisitor)
    {
        this.usageMarker   = usageMarker;
        this.memberVisitor = memberVisitor;
    }


    // Implementations for MemberVisitor.


    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        if (usageMarker.isUsed(programField))
        {
            memberVisitor.visitProgramField(programClass, programField);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (usageMarker.isUsed(programMethod))
        {
            memberVisitor.visitProgramMethod(programClass, programMethod);
        }
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        if (usageMarker.isUsed(libraryField))
        {
            memberVisitor.visitLibraryField(libraryClass, libraryField);
        }
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        if (usageMarker.isUsed(libraryMethod))
        {
            memberVisitor.visitLibraryMethod(libraryClass, libraryMethod);
        }
    }
}
