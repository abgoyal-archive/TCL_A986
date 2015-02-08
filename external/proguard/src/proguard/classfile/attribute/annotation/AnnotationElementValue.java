
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.annotation.visitor.*;

public class AnnotationElementValue extends ElementValue
{
    public Annotation annotationValue;


    /**
     * Creates an uninitialized AnnotationElementValue.
     */
    public AnnotationElementValue()
    {
    }


    /**
     * Creates an initialized AnnotationElementValue.
     */
    public AnnotationElementValue(int        u2elementNameIndex,
                                  Annotation annotationValue)
    {
        super(u2elementNameIndex);

        this.annotationValue = annotationValue;
    }


    /**
     * Applies the given visitor to the annotation.
     */
    public void annotationAccept(Clazz clazz, AnnotationVisitor annotationVisitor)
    {
        annotationVisitor.visitAnnotation(clazz, annotationValue);
    }


    // Implementations for ElementValue.

    public int getTag()
    {
        return ClassConstants.ELEMENT_VALUE_ANNOTATION;
    }

    public void accept(Clazz clazz, Annotation annotation, ElementValueVisitor elementValueVisitor)
    {
        elementValueVisitor.visitAnnotationElementValue(clazz, annotation, this);
    }
}
