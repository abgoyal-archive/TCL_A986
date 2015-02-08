
package proguard.classfile.visitor;

import proguard.classfile.*;

public class ClassVersionFilter implements ClassVisitor
{
    private final int          minimumClassVersion;
    private final int          maximumClassVersion;
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassVersionFilter.
     * @param minimumClassVersion the minimum class version number.
     * @param maximumClassVersion the maximum class version number.
     * @param classVisitor        the <code>ClassVisitor</code> to which visits
     *                            will be delegated.
     */
    public ClassVersionFilter(int          minimumClassVersion,
                              int          maximumClassVersion,
                              ClassVisitor classVisitor)
    {
        this.minimumClassVersion = minimumClassVersion;
        this.maximumClassVersion = maximumClassVersion;
        this.classVisitor        = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (programClass.u4version >= minimumClassVersion &&
            programClass.u4version <= maximumClassVersion)
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Library classes don't have version numbers.
    }
}
