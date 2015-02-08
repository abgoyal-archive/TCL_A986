
package proguard.classfile.visitor;

import proguard.classfile.*;


public class ClassAccessFilter implements ClassVisitor
{
    private final int          requiredSetAccessFlags;
    private final int          requiredUnsetAccessFlags;
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassAccessFilter.
     * @param requiredSetAccessFlags   the class access flags that should be
     *                                 set.
     * @param requiredUnsetAccessFlags the class access flags that should be
     *                                 unset.
     * @param classVisitor             the <code>ClassVisitor</code> to
     *                                 which visits will be delegated.
     */
    public ClassAccessFilter(int          requiredSetAccessFlags,
                             int          requiredUnsetAccessFlags,
                             ClassVisitor classVisitor)
    {
        this.requiredSetAccessFlags   = requiredSetAccessFlags;
        this.requiredUnsetAccessFlags = requiredUnsetAccessFlags;
        this.classVisitor             = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (accepted(programClass.getAccessFlags()))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (accepted(libraryClass.getAccessFlags()))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }


    // Small utility methods.

    private boolean accepted(int accessFlags)
    {
        return (requiredSetAccessFlags   & ~accessFlags) == 0 &&
               (requiredUnsetAccessFlags &  accessFlags) == 0;
    }
}
