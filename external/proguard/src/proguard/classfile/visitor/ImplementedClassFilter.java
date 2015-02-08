
package proguard.classfile.visitor;

import proguard.classfile.*;

public class ImplementedClassFilter implements ClassVisitor
{
    private final Clazz        implementedClass;
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ImplementedClassFilter.
     * @param implementedClass the class whose implementations will not be
     *                         visited.
     * @param classVisitor     the <code>ClassVisitor</code> to which visits will
     *                         be delegated.
     */
    public ImplementedClassFilter(Clazz        implementedClass,
                                  ClassVisitor classVisitor)
    {
        this.implementedClass = implementedClass;
        this.classVisitor     = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (!programClass.extendsOrImplements(implementedClass))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (!libraryClass.extendsOrImplements(implementedClass))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }
}