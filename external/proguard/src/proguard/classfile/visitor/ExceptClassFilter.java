
package proguard.classfile.visitor;

import proguard.classfile.*;

public class ExceptClassFilter implements ClassVisitor
{
    private final Clazz        exceptClass;
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassNameFilter.
     * @param exceptClass  the class that will not be visited.
     * @param classVisitor the <code>ClassVisitor</code> to which visits will
     *                     be delegated.
     */
    public ExceptClassFilter(Clazz        exceptClass,
                             ClassVisitor classVisitor)
    {
        this.exceptClass  = exceptClass;
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (!programClass.equals(exceptClass))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (!libraryClass.equals(exceptClass))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }
}