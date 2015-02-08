
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.annotation.visitor.ElementValueVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class ClassElementValue extends ElementValue
{
    public int u2classInfoIndex;

    /**
     * An extra field pointing to the Clazz objects referenced in the
     * type name string. This field is filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz[] referencedClasses;


    /**
     * Creates an uninitialized ClassElementValue.
     */
    public ClassElementValue()
    {
    }


    /**
     * Creates an initialized ClassElementValue.
     */
    public ClassElementValue(int u2elementNameIndex,
                             int u2classInfoIndex)
    {
        super(u2elementNameIndex);

        this.u2classInfoIndex = u2classInfoIndex;
    }


    /**
     * Applies the given visitor to all referenced classes.
     */
    public void referencedClassesAccept(ClassVisitor classVisitor)
    {
        if (referencedClasses != null)
        {
            for (int index = 0; index < referencedClasses.length; index++)
            {
                Clazz referencedClass = referencedClasses[index];
                if (referencedClass != null)
                {
                    referencedClass.accept(classVisitor);
                }
            }
        }
    }


    // Implementations for ElementValue.

    public int getTag()
    {
        return ClassConstants.ELEMENT_VALUE_CLASS;
    }

    public void accept(Clazz clazz, Annotation annotation, ElementValueVisitor elementValueVisitor)
    {
        elementValueVisitor.visitClassElementValue(clazz, annotation, this);
    }
}
