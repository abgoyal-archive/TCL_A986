
package proguard.classfile.util;

import proguard.classfile.*;
import proguard.classfile.visitor.ClassVisitor;

public class ClassSubHierarchyInitializer
implements   ClassVisitor
{
    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Add this class to the subclasses of its superclass.
        addSubclass(programClass, programClass.getSuperClass());

        // Add this class to the subclasses of its interfaces.
        for (int index = 0; index < programClass.u2interfacesCount; index++)
        {
            addSubclass(programClass, programClass.getInterface(index));
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Add this class to the subclasses of its superclass,
        addSubclass(libraryClass, libraryClass.superClass);

        // Add this class to the subclasses of its interfaces.
        Clazz[] interfaceClasses = libraryClass.interfaceClasses;
        if (interfaceClasses != null)
        {
            for (int index = 0; index < interfaceClasses.length; index++)
            {
                // Add this class to the subclasses of the interface class.
                addSubclass(libraryClass, interfaceClasses[index]);
            }
        }
    }


    // Small utility methods.

    private void addSubclass(Clazz subclass, Clazz clazz)
    {
        if (clazz != null)
        {
            clazz.addSubClass(subclass);
        }
    }
}
