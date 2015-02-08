
package proguard.classfile.visitor;

import proguard.classfile.*;

public class ClassCounter implements ClassVisitor
{
    private int count;


    /**
     * Returns the number of classes that has been visited so far.
     */
    public int getCount()
    {
        return count;
    }


    // Implementations for ClassVisitor.

    public void visitLibraryClass(LibraryClass libraryClass)
    {
        count++;
    }


    public void visitProgramClass(ProgramClass programClass)
    {
        count++;
    }
}
