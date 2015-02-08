
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.Attribute;
import proguard.classfile.attribute.annotation.visitor.AnnotationVisitor;

public abstract class ParameterAnnotationsAttribute extends Attribute
{
    public int            u2parametersCount;
    public int[]          u2parameterAnnotationsCount;
    public Annotation[][] parameterAnnotations;


    /**
     * Creates an uninitialized ParameterAnnotationsAttribute.
     */
    protected ParameterAnnotationsAttribute()
    {
    }


    /**
     * Creates an initialized ParameterAnnotationsAttribute.
     */
    protected ParameterAnnotationsAttribute(int            u2attributeNameIndex,
                                            int            u2parametersCount,
                                            int[]          u2parameterAnnotationsCount,
                                            Annotation[][] parameterAnnotations)
    {
        super(u2attributeNameIndex);

        this.u2parametersCount           = u2parametersCount;
        this.u2parameterAnnotationsCount = u2parameterAnnotationsCount;
        this.parameterAnnotations        = parameterAnnotations;
    }


    /**
     * Applies the given visitor to all annotations.
     */
    public void annotationsAccept(Clazz clazz, Method method, AnnotationVisitor annotationVisitor)
    {
        // Loop over all parameters.
        for (int parameterIndex = 0; parameterIndex < u2parametersCount; parameterIndex++)
        {
            int          annotationsCount = u2parameterAnnotationsCount[parameterIndex];
            Annotation[] annotations      = parameterAnnotations[parameterIndex];

            // Loop over all parameter annotations.
            for (int index = 0; index < annotationsCount; index++)
            {
                // We don't need double dispatching here, since there is only one
                // type of Annotation.
                annotationVisitor.visitAnnotation(clazz, method, parameterIndex, annotations[index]);
            }
        }
    }
}
