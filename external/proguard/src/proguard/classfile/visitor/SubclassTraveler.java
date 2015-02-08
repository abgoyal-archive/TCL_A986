
package proguard.classfile.visitor;

import proguard.classfile.*;


public class SubclassTraveler implements ClassVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassHierarchyTraveler.
     * @param classVisitor    the <code>ClassVisitor</code> to
     *                        which visits will be delegated.
     */
    public SubclassTraveler(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        programClass.subclassesAccept(classVisitor);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        libraryClass.subclassesAccept(classVisitor);
    }
}