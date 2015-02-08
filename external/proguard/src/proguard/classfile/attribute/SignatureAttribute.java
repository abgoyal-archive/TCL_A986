
package proguard.classfile.attribute;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class SignatureAttribute extends Attribute
{
    public int u2signatureIndex;

    /**
     * An extra field pointing to the Clazz objects referenced in the
     * signature string. This field is filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz[] referencedClasses;


    /**
     * Creates an uninitialized SignatureAttribute.
     */
    public SignatureAttribute()
    {
    }


    /**
     * Creates an initialized SignatureAttribute.
     */
    public SignatureAttribute(int u2attributeNameIndex,
                              int u2signatureIndex)
    {
        super(u2attributeNameIndex);

        this.u2signatureIndex = u2signatureIndex;
    }


    /**
     * Lets the Clazz objects referenced in the signature string accept the
     * given visitor.
     */
    public void referencedClassesAccept(ClassVisitor classVisitor)
    {
        if (referencedClasses != null)
        {
            for (int index = 0; index < referencedClasses.length; index++)
            {
                if (referencedClasses[index] != null)
                {
                    referencedClasses[index].accept(classVisitor);
                }
            }
        }
    }


    // Implementations for Attribute.

    public void accept(Clazz clazz, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitSignatureAttribute(clazz, this);
    }

    public void accept(Clazz clazz, Field field, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitSignatureAttribute(clazz, field, this);
    }

    public void accept(Clazz clazz, Method method, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitSignatureAttribute(clazz, method, this);
    }
 }
