
package proguard.classfile.visitor;

import proguard.classfile.*;


public class ProgramClassFilter implements ClassVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ProgramClassFilter.
     * @param classVisitor     the <code>ClassVisitor</code> to which visits
     *                         will be delegated.
     */
    public ProgramClassFilter(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        classVisitor.visitProgramClass(programClass);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Don't delegate visits to library classes.
    }
}
