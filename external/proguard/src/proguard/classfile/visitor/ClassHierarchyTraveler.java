
package proguard.classfile.visitor;

import proguard.classfile.*;


public class ClassHierarchyTraveler implements ClassVisitor
{
    private final boolean visitThisClass;
    private final boolean visitSuperClass;
    private final boolean visitInterfaces;
    private final boolean visitSubclasses;

    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassHierarchyTraveler.
     * @param visitThisClass  specifies whether to visit the originally visited
     *                        classes.
     * @param visitSuperClass specifies whether to visit the super classes of
     *                        the visited classes.
     * @param visitInterfaces specifies whether to visit the interfaces of
     *                        the visited classes.
     * @param visitSubclasses specifies whether to visit the subclasses of
     *                        the visited classes.
     * @param classVisitor    the <code>ClassVisitor</code> to
     *                        which visits will be delegated.
     */
    public ClassHierarchyTraveler(boolean      visitThisClass,
                                  boolean      visitSuperClass,
                                  boolean      visitInterfaces,
                                  boolean      visitSubclasses,
                                  ClassVisitor classVisitor)
    {
        this.visitThisClass  = visitThisClass;
        this.visitSuperClass = visitSuperClass;
        this.visitInterfaces = visitInterfaces;
        this.visitSubclasses = visitSubclasses;

        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        programClass.hierarchyAccept(visitThisClass,
                                     visitSuperClass,
                                     visitInterfaces,
                                     visitSubclasses,
                                     classVisitor);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        libraryClass.hierarchyAccept(visitThisClass,
                                     visitSuperClass,
                                     visitInterfaces,
                                     visitSubclasses,
                                     classVisitor);
    }
}
