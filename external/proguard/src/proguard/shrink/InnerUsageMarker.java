
package proguard.shrink;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.*;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class InnerUsageMarker
extends      SimplifiedVisitor
implements   AttributeVisitor,
             InnerClassesInfoVisitor,
             ConstantVisitor,
             ClassVisitor
{
    private final UsageMarker usageMarker;

    // Fields acting as a return parameters for several methods.
    private boolean attributeUsed;
    private boolean classUsed;


    /**
     * Creates a new InnerUsageMarker.
     * @param usageMarker the usage marker that is used to mark the classes
     *                    and class members.
     */
    public InnerUsageMarker(UsageMarker usageMarker)
    {
        this.usageMarker = usageMarker;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitInnerClassesAttribute(Clazz clazz, InnerClassesAttribute innerClassesAttribute)
    {
        // Mark the necessary inner classes information.
        attributeUsed = false;
        innerClassesAttribute.innerClassEntriesAccept(clazz, this);

        if (attributeUsed)
        {
            // We got a positive used flag, so some inner class is being used.
            // Mark this attribute as being used as well.
            usageMarker.markAsUsed(innerClassesAttribute);

            markConstant(clazz, innerClassesAttribute.u2attributeNameIndex);
        }
    }


    // Implementations for InnerClassesInfoVisitor.

    public void visitInnerClassesInfo(Clazz clazz, InnerClassesInfo innerClassesInfo)
    {
        boolean innerClassesInfoUsed = usageMarker.isUsed(innerClassesInfo);

        if (!innerClassesInfoUsed)
        {
            // Check if the inner class (if any) is marked as being used.
            classUsed = true;
            innerClassesInfo.innerClassConstantAccept(clazz, this);
            innerClassesInfoUsed = classUsed;

            // Check if the outer class (if any) is marked as being used.
            classUsed = true;
            innerClassesInfo.outerClassConstantAccept(clazz, this);
            innerClassesInfoUsed &= classUsed;

            // If both the inner class and the outer class are marked as being
            // used, then mark this InnerClassesInfo as well.
            if (innerClassesInfoUsed)
            {
                usageMarker.markAsUsed(innerClassesInfo);

                innerClassesInfo.innerNameConstantAccept(clazz, this);
            }
        }

        // The return value.
        attributeUsed |= innerClassesInfoUsed;
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        classUsed = usageMarker.isUsed(classConstant);

        // Is the class constant marked as being used?
        if (!classUsed)
        {
            // Check the referenced class.
            classUsed = true;
            classConstant.referencedClassAccept(this);

            // Is the referenced class marked as being used?
            if (classUsed)
            {
                // Mark the class constant and its Utf8 constant.
                usageMarker.markAsUsed(classConstant);

                markConstant(clazz, classConstant.u2nameIndex);
            }
        }
    }


    public void visitUtf8Constant(Clazz clazz, Utf8Constant utf8Constant)
    {
        usageMarker.markAsUsed(utf8Constant);
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        classUsed = usageMarker.isUsed(programClass);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        classUsed = true;
    }


    // Small utility methods.

    /**
     * Marks the given constant pool entry of the given class. This includes
     * visiting any other referenced constant pool entries.
     */
    private void markConstant(Clazz clazz, int index)
    {
         clazz.constantPoolEntryAccept(index, this);
    }
}
