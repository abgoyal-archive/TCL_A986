
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.Attribute;
import proguard.classfile.attribute.annotation.visitor.ElementValueVisitor;
import proguard.classfile.attribute.visitor.AttributeVisitor;

public class AnnotationDefaultAttribute extends Attribute
{
    public ElementValue defaultValue;


    /**
     * Creates an uninitialized AnnotationDefaultAttribute.
     */
    public AnnotationDefaultAttribute()
    {
    }


    /**
     * Creates an initialized AnnotationDefaultAttribute.
     */
    public AnnotationDefaultAttribute(int          u2attributeNameIndex,
                                      ElementValue defaultValue)
    {
        super(u2attributeNameIndex);

        this.defaultValue = defaultValue;
    }


    /**
     * Applies the given visitor to the default element value.
     */
    public void defaultValueAccept(Clazz clazz, ElementValueVisitor elementValueVisitor)
    {
        defaultValue.accept(clazz, null, elementValueVisitor);
    }


    // Implementations for Attribute.

    public void accept(Clazz clazz, Method method, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitAnnotationDefaultAttribute(clazz, method, this);
    }
}
