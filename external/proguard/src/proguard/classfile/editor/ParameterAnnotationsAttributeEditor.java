
package proguard.classfile.editor;

import proguard.classfile.attribute.annotation.*;

public class ParameterAnnotationsAttributeEditor
{
    private ParameterAnnotationsAttribute targetParameterAnnotationsAttribute;


    /**
     * Creates a new ParameterAnnotationsAttributeEditor that will edit
     * annotations in the given parameter annotations attribute.
     */
    public ParameterAnnotationsAttributeEditor(ParameterAnnotationsAttribute targetParameterAnnotationsAttribute)
    {
        this.targetParameterAnnotationsAttribute = targetParameterAnnotationsAttribute;
    }


    /**
     * Adds a given annotation to the annotations attribute.
     */
    public void addAnnotation(int parameterIndex, Annotation annotation)
    {
        int          annotationsCount = targetParameterAnnotationsAttribute.u2parameterAnnotationsCount[parameterIndex];
        Annotation[] annotations      = targetParameterAnnotationsAttribute.parameterAnnotations[parameterIndex];

        // Make sure there is enough space for the new annotation.
        if (annotations == null ||
            annotations.length <= annotationsCount)
        {
            targetParameterAnnotationsAttribute.parameterAnnotations[parameterIndex] = new Annotation[annotationsCount+1];
            if (annotations != null)
            {
                System.arraycopy(annotations, 0,
                                 targetParameterAnnotationsAttribute.parameterAnnotations[parameterIndex], 0,
                                 annotationsCount);
            }
            annotations = targetParameterAnnotationsAttribute.parameterAnnotations[parameterIndex];
        }

        // Add the annotation.
        annotations[targetParameterAnnotationsAttribute.u2parameterAnnotationsCount[parameterIndex]++] = annotation;
    }
}