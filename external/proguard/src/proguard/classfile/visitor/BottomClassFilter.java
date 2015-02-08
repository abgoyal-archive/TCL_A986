
package proguard.classfile.visitor;

import proguard.classfile.*;


public class BottomClassFilter implements ClassVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ProgramClassFilter.
     * @param classVisitor     the <code>ClassVisitor</code> to which visits
     *                         will be delegated.
     */
    public BottomClassFilter(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Is this a bottom class in the class hierarchy?
        if (programClass.subClasses == null)
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Is this a bottom class in the class hierarchy?
        if (libraryClass.subClasses == null)
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }
}
