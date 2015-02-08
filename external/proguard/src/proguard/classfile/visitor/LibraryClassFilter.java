
package proguard.classfile.visitor;

import proguard.classfile.*;


public class LibraryClassFilter implements ClassVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new LibraryClassFilter.
     * @param classVisitor     the <code>ClassVisitor</code> to which visits
     *                         will be delegated.
     */
    public LibraryClassFilter(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Don't delegate visits to program classes.
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        classVisitor.visitLibraryClass(libraryClass);
    }
}
