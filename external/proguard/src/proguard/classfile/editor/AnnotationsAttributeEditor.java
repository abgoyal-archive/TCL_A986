
package proguard.classfile.editor;

import proguard.classfile.attribute.annotation.*;

public class AnnotationsAttributeEditor
{
    private AnnotationsAttribute targetAnnotationsAttribute;


    /**
     * Creates a new AnnotationsAttributeEditor that will edit annotations in
     * the given annotations attribute.
     */
    public AnnotationsAttributeEditor(AnnotationsAttribute targetAnnotationsAttribute)
    {
        this.targetAnnotationsAttribute = targetAnnotationsAttribute;
    }


    /**
     * Adds a given annotation to the annotations attribute.
     */
    public void addAnnotation(Annotation annotation)
    {
        int          annotationsCount = targetAnnotationsAttribute.u2annotationsCount;
        Annotation[] annotations      = targetAnnotationsAttribute.annotations;

        // Make sure there is enough space for the new annotation.
        if (annotations.length <= annotationsCount)
        {
            targetAnnotationsAttribute.annotations = new Annotation[annotationsCount+1];
            System.arraycopy(annotations, 0,
                             targetAnnotationsAttribute.annotations, 0,
                             annotationsCount);
            annotations = targetAnnotationsAttribute.annotations;
        }

        // Add the annotation.
        annotations[targetAnnotationsAttribute.u2annotationsCount++] = annotation;
    }
}