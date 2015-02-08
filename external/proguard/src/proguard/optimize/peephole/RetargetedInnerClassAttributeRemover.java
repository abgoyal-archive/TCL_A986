
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.*;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class RetargetedInnerClassAttributeRemover
extends      SimplifiedVisitor
implements   ClassVisitor,
             AttributeVisitor,
             InnerClassesInfoVisitor,
             ConstantVisitor
{
    private boolean retargeted;


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        int         attributesCount = programClass.u2attributesCount;
        Attribute[] attributes      = programClass.attributes;

        int newAtributesCount = 0;

        // Copy over all non-retargeted attributes.
        for (int index = 0; index < attributesCount; index++)
        {
            Attribute attribute = attributes[index];

            // Check if it's an InnerClasses or EnclosingMethod attribute in
            // a retargeted class or referring to a retargeted class.
            retargeted = false;
            attribute.accept(programClass, this);
            if (!retargeted)
            {
                attributes[newAtributesCount++] = attribute;
            }
        }

        // Clean up any remaining array elements.
        for (int index = newAtributesCount; index < attributesCount; index++)
        {
            attributes[index] = null;
        }

        // Update the number of attribuets.
        programClass.u2attributesCount = newAtributesCount;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitInnerClassesAttribute(Clazz clazz, InnerClassesAttribute innerClassesAttribute)
    {
        // Check whether the class itself is retargeted.
        checkTarget(clazz);

        // Check whether the referenced classes are retargeted.
        innerClassesAttribute.innerClassEntriesAccept(clazz, this);
    }


    public void visitEnclosingMethodAttribute(Clazz clazz, EnclosingMethodAttribute enclosingMethodAttribute)
    {
        // Check whether the class itself is retargeted.
        checkTarget(clazz);

        // Check whether the referenced class is retargeted.
        checkTarget(enclosingMethodAttribute.referencedClass);
    }


    // Implementations for InnerClassesInfoVisitor.

    public void visitInnerClassesInfo(Clazz clazz, InnerClassesInfo innerClassesInfo)
    {
        // Check whether the inner class or the outer class are retargeted.
        innerClassesInfo.innerClassConstantAccept(clazz, this);
        innerClassesInfo.outerClassConstantAccept(clazz, this);
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        // Check whether the referenced class is retargeted.
        checkTarget(classConstant.referencedClass);
    }


    // Small utility methods.

    /**
     * Sets the global return value to true if the given class is retargeted.
     */
    private void checkTarget(Clazz clazz)
    {
        if (clazz != null &&
            ClassMerger.getTargetClass(clazz) != null)
        {
            retargeted = true;
        }
    }
}