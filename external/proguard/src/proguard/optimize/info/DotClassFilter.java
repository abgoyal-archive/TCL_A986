
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

public class DotClassFilter
implements   ClassVisitor
{
    private final ClassVisitor classVisitor;


    public DotClassFilter(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (DotClassMarker.isDotClassed(programClass))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (DotClassMarker.isDotClassed(libraryClass))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }
}