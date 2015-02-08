
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.annotation.visitor.ElementValueVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class EnumConstantElementValue extends ElementValue
{
    public int u2typeNameIndex;
    public int u2constantNameIndex;

    /**
     * An extra field pointing to the Clazz objects referenced in the
     * type name string. This field is typically filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer
     * ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz[] referencedClasses;


    /**
     * Creates an uninitialized EnumConstantElementValue.
     */
    public EnumConstantElementValue()
    {
    }


    /**
     * Creates an initialized EnumConstantElementValue.
     */
    public EnumConstantElementValue(int u2elementNameIndex,
                                    int u2typeNameIndex,
                                    int u2constantNameIndex)
    {
        super(u2elementNameIndex);

        this.u2typeNameIndex     = u2typeNameIndex;
        this.u2constantNameIndex = u2constantNameIndex;
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
        return ClassConstants.ELEMENT_VALUE_ENUM_CONSTANT;
    }

    public void accept(Clazz clazz, Annotation annotation, ElementValueVisitor elementValueVisitor)
    {
        elementValueVisitor.visitEnumConstantElementValue(clazz, annotation, this);
    }
}
