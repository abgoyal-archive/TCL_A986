
package proguard.shrink;

import proguard.classfile.*;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;


public class InterfaceUsageMarker
extends      SimplifiedVisitor
implements   ClassVisitor,
             ConstantVisitor
{
    private final UsageMarker usageMarker;

    // Fields acting as a return parameters for several methods.
    private boolean used;
    private boolean anyUsed;


    /**
     * Creates a new InterfaceUsageMarker.
     * @param usageMarker the usage marker that is used to mark the classes
     *                    and class members.
     */
    public InterfaceUsageMarker(UsageMarker usageMarker)
    {
        this.usageMarker = usageMarker;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        boolean classUsed         = usageMarker.isUsed(programClass);
        boolean classPossiblyUsed = usageMarker.isPossiblyUsed(programClass);

        if (classUsed || classPossiblyUsed)
        {
            // Check if any interfaces are being used.
            boolean oldAnyUsed = anyUsed;
            anyUsed = false;

            programClass.interfaceConstantsAccept(this);

            classUsed |= anyUsed;
            anyUsed = oldAnyUsed;

            // Is this an interface with a preliminary mark?
            if (classPossiblyUsed)
            {
                // Should it be included now?
                if (classUsed)
                {
                    // At least one if this interface's interfaces is being used.
                    // Mark this interface as well.
                    usageMarker.markAsUsed(programClass);

                    // Mark this interface's name.
                    programClass.thisClassConstantAccept(this);

                    // Mark the superclass (java/lang/Object).
                    programClass.superClassConstantAccept(this);
                }
                else
                {
                    // Unmark this interface, so we don't bother looking at it again.
                    usageMarker.markAsUnused(programClass);
                }
            }
        }

        // The return value.
        used = classUsed;
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // The return values.
        used    = true;
        anyUsed = true;
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        boolean classUsed = usageMarker.isUsed(classConstant);

        if (!classUsed)
        {
            // The ClassConstant isn't marked as being used yet. But maybe it
            // should be included as an interface, so check the actual class.
            classConstant.referencedClassAccept(this);
            classUsed = used;

            if (classUsed)
            {
                // The class is being used. Mark the ClassConstant as being used
                // as well.
                usageMarker.markAsUsed(classConstant);

                clazz.constantPoolEntryAccept(classConstant.u2nameIndex, this);
            }
        }

        // The return values.
        used    =  classUsed;
        anyUsed |= classUsed;
    }


    public void visitUtf8Constant(Clazz clazz, Utf8Constant utf8Constant)
    {
        if (!usageMarker.isUsed(utf8Constant))
        {
            usageMarker.markAsUsed(utf8Constant);
        }
    }
}
