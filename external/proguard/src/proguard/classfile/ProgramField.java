
package proguard.classfile;

import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.attribute.Attribute;
import proguard.classfile.visitor.*;

public class ProgramField extends ProgramMember implements Field
{
    /**
     * An extra field pointing to the Clazz object referenced in the
     * descriptor string. This field is filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz referencedClass;


    /**
     * Creates an uninitialized ProgramField.
     */
    public ProgramField()
    {
    }


    /**
     * Creates an initialized ProgramField.
     */
    public ProgramField(int         u2accessFlags,
                        int         u2nameIndex,
                        int         u2descriptorIndex,
                        int         u2attributesCount,
                        Attribute[] attributes,
                        Clazz       referencedClass)
    {
        super(u2accessFlags, u2nameIndex, u2descriptorIndex, u2attributesCount, attributes);

        this.referencedClass = referencedClass;
    }


    // Implementations for ProgramMember.

    public void accept(ProgramClass programClass, MemberVisitor memberVisitor)
    {
        memberVisitor.visitProgramField(programClass, this);
    }


    public void attributesAccept(ProgramClass programClass, AttributeVisitor attributeVisitor)
    {
        for (int index = 0; index < u2attributesCount; index++)
        {
            attributes[index].accept(programClass, this, attributeVisitor);
        }
    }


    // Implementations for Member.

    public void referencedClassesAccept(ClassVisitor classVisitor)
    {
        if (referencedClass != null)
        {
            referencedClass.accept(classVisitor);
        }
    }
}
