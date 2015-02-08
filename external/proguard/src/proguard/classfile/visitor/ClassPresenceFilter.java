
package proguard.classfile.visitor;

import proguard.classfile.*;

public class ClassPresenceFilter implements ClassVisitor
{
    private final ClassPool    classPool;
    private final ClassVisitor presentClassVisitor;
    private final ClassVisitor missingClassVisitor;


    /**
     * Creates a new ClassPresenceFilter.
     * @param classPool           the <code>ClassPool</code> in which the
     *                            presence will be tested.
     * @param presentClassVisitor the <code>ClassVisitor</code> to which visits
     *                            of present class files will be delegated.
     * @param missingClassVisitor the <code>ClassVisitor</code> to which visits
     *                            of missing class files will be delegated.
     */
    public ClassPresenceFilter(ClassPool    classPool,
                               ClassVisitor presentClassVisitor,
                               ClassVisitor missingClassVisitor)
    {
        this.classPool           = classPool;
        this.presentClassVisitor = presentClassVisitor;
        this.missingClassVisitor = missingClassVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        ClassVisitor classFileVisitor = classFileVisitor(programClass);

        if (classFileVisitor != null)
        {
            classFileVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        ClassVisitor classFileVisitor = classFileVisitor(libraryClass);

        if (classFileVisitor != null)
        {
            classFileVisitor.visitLibraryClass(libraryClass);
        }
    }


    // Small utility methods.

    /**
     * Returns the appropriate <code>ClassVisitor</code>.
     */
    private ClassVisitor classFileVisitor(Clazz clazz)
    {
        return classPool.getClass(clazz.getName()) != null ?
            presentClassVisitor :
            missingClassVisitor;
    }
}
