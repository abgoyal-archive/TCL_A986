
package proguard.classfile.visitor;

import proguard.classfile.*;

public class ExceptClassesFilter implements ClassVisitor
{
    private final Clazz[]      exceptClasses;
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ExceptClassesFilter.
     * @param exceptClasses the classes that will not be visited.
     * @param classVisitor  the <code>ClassVisitor</code> to which visits will
     *                      be delegated.
     */
    public ExceptClassesFilter(Clazz[]      exceptClasses,
                               ClassVisitor classVisitor)
    {
        this.exceptClasses = exceptClasses;
        this.classVisitor  = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (!present(programClass))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (!present(libraryClass))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }


    // Small utility methods.

    private boolean present(Clazz clazz)
    {
        if (exceptClasses == null)
        {
            return false;
        }

        for (int index = 0; index < exceptClasses.length; index++)
        {
            if (exceptClasses[index].equals(clazz))
            {
                return true;
            }
        }

        return false;
    }
}