
package proguard.classfile.util;

import proguard.classfile.*;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class ClassSuperHierarchyInitializer
extends      SimplifiedVisitor
implements   ClassVisitor,
             ConstantVisitor
{
    private final ClassPool      programClassPool;
    private final ClassPool      libraryClassPool;
    private final WarningPrinter missingWarningPrinter;
    private final WarningPrinter dependencyWarningPrinter;


    /**
     * Creates a new ClassSuperHierarchyInitializer that initializes the super
     * hierarchy of all visited class files, optionally printing warnings if
     * some classes can't be found or if they are in the program class pool.
     */
    public ClassSuperHierarchyInitializer(ClassPool      programClassPool,
                                          ClassPool      libraryClassPool,
                                          WarningPrinter missingWarningPrinter,
                                          WarningPrinter dependencyWarningPrinter)
    {
        this.programClassPool         = programClassPool;
        this.libraryClassPool         = libraryClassPool;
        this.missingWarningPrinter    = missingWarningPrinter;
        this.dependencyWarningPrinter = dependencyWarningPrinter;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Link to the super class.
        programClass.superClassConstantAccept(this);

        // Link to the interfaces.
        programClass.interfaceConstantsAccept(this);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        String className = libraryClass.getName();

        // Link to the super class.
        String superClassName = libraryClass.superClassName;
        if (superClassName != null)
        {
            // Keep a reference to the superclass.
            libraryClass.superClass = findClass(className, superClassName);
        }

        // Link to the interfaces.
        if (libraryClass.interfaceNames != null)
        {
            String[] interfaceNames   = libraryClass.interfaceNames;
            Clazz[]  interfaceClasses = new Clazz[interfaceNames.length];

            for (int index = 0; index < interfaceNames.length; index++)
            {
                // Keep a reference to the interface class.
                interfaceClasses[index] =
                    findClass(className, interfaceNames[index]);
            }

            libraryClass.interfaceClasses = interfaceClasses;
        }

        // Discard the name Strings. From now on, we'll use the object
        // references.
        libraryClass.superClassName = null;
        libraryClass.interfaceNames = null;
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        classConstant.referencedClass =
            findClass(clazz.getName(), classConstant.getName(clazz));
    }


    // Small utility methods.

    /**
     * Returns the class with the given name, either for the program class pool
     * or from the library class pool, or <code>null</code> if it can't be found.
     */
    private Clazz findClass(String referencingClassName, String name)
    {
        // First look for the class in the program class pool.
        Clazz clazz = programClassPool.getClass(name);

        // Otherwise look for the class in the library class pool.
        if (clazz == null)
        {
            clazz = libraryClassPool.getClass(name);

            if (clazz == null &&
                missingWarningPrinter != null)
            {
                // We didn't find the superclass or interface. Print a warning.
                missingWarningPrinter.print(referencingClassName,
                                            name,
                                            "Warning: " +
                                            ClassUtil.externalClassName(referencingClassName) +
                                            ": can't find superclass or interface " +
                                            ClassUtil.externalClassName(name));
            }
        }
        else if (dependencyWarningPrinter != null)
        {
            // The superclass or interface was found in the program class pool.
            // Print a warning.
            dependencyWarningPrinter.print(referencingClassName,
                                           name,
                                           "Warning: library class " +
                                           ClassUtil.externalClassName(referencingClassName) +
                                           " extends or implements program class " +
                                           ClassUtil.externalClassName(name));
        }

        return clazz;
    }
}
