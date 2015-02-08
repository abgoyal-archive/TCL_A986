
package proguard.classfile.attribute.annotation;

import proguard.classfile.*;
import proguard.classfile.attribute.annotation.visitor.ElementValueVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class Annotation implements VisitorAccepter
{
    public int            u2typeIndex;
    public int            u2elementValuesCount;
    public ElementValue[] elementValues;

    /**
     * An extra field pointing to the Clazz objects referenced in the
     * type string. This field is typically filled out by the <code>{@link
     * proguard.classfile.util.ClassReferenceInitializer
     * ClassReferenceInitializer}</code>.
     * References to primitive types are ignored.
     */
    public Clazz[] referencedClasses;

    /**
     * An extra field in which visitors can store information.
     */
    public Object visitorInfo;


    /**
     * Creates an uninitialized Annotation.
     */
    public Annotation()
    {
    }


    /**
     * Creates an initialized Annotation.
     */
    public Annotation(int            u2typeIndex,
                      int            u2elementValuesCount,
                      ElementValue[] elementValues)
    {
        this.u2typeIndex          = u2typeIndex;
        this.u2elementValuesCount = u2elementValuesCount;
        this.elementValues        = elementValues;
    }


    /**
     * Returns the type.
     */
    public String getType(Clazz clazz)
    {
        return clazz.getString(u2typeIndex);
    }



    /**
     * Applies the given visitor to the first referenced class. This is the
     * main annotation class.
     */
    public void referencedClassAccept(ClassVisitor classVisitor)
    {
        if (referencedClasses != null)
        {
            Clazz referencedClass = referencedClasses[0];
            if (referencedClass != null)
            {
                referencedClass.accept(classVisitor);
            }
        }
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


    /**
     * Applies the given visitor to all element value pairs.
     */
    public void elementValuesAccept(Clazz clazz, ElementValueVisitor elementValueVisitor)
    {
        for (int index = 0; index < u2elementValuesCount; index++)
        {
            elementValues[index].accept(clazz, this, elementValueVisitor);
        }
    }


    // Implementations for VisitorAccepter.

    public Object getVisitorInfo()
    {
        return visitorInfo;
    }

    public void setVisitorInfo(Object visitorInfo)
    {
        this.visitorInfo = visitorInfo;
    }
}
