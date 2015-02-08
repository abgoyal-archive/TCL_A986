
package proguard.classfile.visitor;

import proguard.classfile.*;


public class ConcreteClassDownTraveler
implements   ClassVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ConcreteClassDownTraveler.
     * @param classVisitor     the <code>ClassVisitor</code> to
     *                         which visits will be delegated.
     */
    public ConcreteClassDownTraveler(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Is this an abstract class or an interface?
        if ((programClass.getAccessFlags() &
             (ClassConstants.INTERNAL_ACC_INTERFACE |
              ClassConstants.INTERNAL_ACC_ABSTRACT)) != 0)
        {
            // Travel down the hierarchy.
            Clazz[] subClasses = programClass.subClasses;
            if (subClasses != null)
            {
                for (int index = 0; index < subClasses.length; index++)
                {
                    subClasses[index].accept(this);
                }
            }
        }
        else
        {
            // Visit the class. Don't descend any further.
            programClass.accept(classVisitor);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Is this an abstract class or interface?
        if ((libraryClass.getAccessFlags() &
             (ClassConstants.INTERNAL_ACC_INTERFACE |
              ClassConstants.INTERNAL_ACC_ABSTRACT)) != 0)
        {
            // Travel down the hierarchy.
            Clazz[] subClasses = libraryClass.subClasses;
            if (subClasses != null)
            {
                for (int index = 0; index < subClasses.length; index++)
                {
                    subClasses[index].accept(this);
                }
            }
        }
        else
        {
            // Visit the class. Don't descend any further.
            libraryClass.accept(classVisitor);
        }
    }
}
