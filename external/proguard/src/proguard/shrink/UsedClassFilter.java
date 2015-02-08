
package proguard.shrink;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

public class UsedClassFilter
implements   ClassVisitor
{
    private final UsageMarker  usageMarker;
    private final ClassVisitor classVisitor;


    /**
     * Creates a new UsedClassFilter.
     * @param usageMarker  the usage marker that is used to mark the classes
     *                     and class members.
     * @param classVisitor the class visitor to which the visiting will be
     *                     delegated.
     */
    public UsedClassFilter(UsageMarker  usageMarker,
                           ClassVisitor classVisitor)
    {
        this.usageMarker  = usageMarker;
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (usageMarker.isUsed(programClass))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (usageMarker.isUsed(libraryClass))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }
}
