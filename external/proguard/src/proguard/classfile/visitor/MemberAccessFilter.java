
package proguard.classfile.visitor;

import proguard.classfile.*;


public class MemberAccessFilter
implements   MemberVisitor
{
    // A mask of conflicting access flags. These are interpreted in a special
    // way if more of them are required at the same time. In that case, one
    // of them being set is sufficient.
    private static final int ACCESS_MASK =
        ClassConstants.INTERNAL_ACC_PUBLIC  |
        ClassConstants.INTERNAL_ACC_PRIVATE |
        ClassConstants.INTERNAL_ACC_PROTECTED;

    private final int           requiredSetAccessFlags;
    private final int           requiredUnsetAccessFlags;
    private final int           requiredOneSetAccessFlags;
    private final MemberVisitor memberVisitor;


    /**
     * Creates a new MemberAccessFilter.
     * @param requiredSetAccessFlags   the class access flags that should be
     *                                 set.
     * @param requiredUnsetAccessFlags the class access flags that should be
     *                                 unset.
     * @param memberVisitor            the <code>MemberVisitor</code> to
     *                                 which visits will be delegated.
     */
    public MemberAccessFilter(int           requiredSetAccessFlags,
                              int           requiredUnsetAccessFlags,
                              MemberVisitor memberVisitor)
    {
        this.requiredSetAccessFlags    = requiredSetAccessFlags & ~ACCESS_MASK;
        this.requiredUnsetAccessFlags  = requiredUnsetAccessFlags;
        this.requiredOneSetAccessFlags = requiredSetAccessFlags &  ACCESS_MASK;
        this.memberVisitor             = memberVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        if (accepted(programField.getAccessFlags()))
        {
            memberVisitor.visitProgramField(programClass, programField);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (accepted(programMethod.getAccessFlags()))
        {
            memberVisitor.visitProgramMethod(programClass, programMethod);
        }
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        if (accepted(libraryField.getAccessFlags()))
        {
            memberVisitor.visitLibraryField(libraryClass, libraryField);
        }
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        if (accepted(libraryMethod.getAccessFlags()))
        {
            memberVisitor.visitLibraryMethod(libraryClass, libraryMethod);
        }
    }


    // Small utility methods.

    private boolean accepted(int accessFlags)
    {
        return (requiredSetAccessFlags    & ~accessFlags) == 0 &&
               (requiredUnsetAccessFlags  &  accessFlags) == 0 &&
               (requiredOneSetAccessFlags == 0                 ||
               (requiredOneSetAccessFlags &  accessFlags) != 0);
    }
}
