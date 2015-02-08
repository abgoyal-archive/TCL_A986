
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.attribute.Attribute;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class AttributeUsageMarker
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    // A visitor info flag to indicate the attribute is being used.
    private static final Object USED = new Object();


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute)
    {
        markAsUsed(attribute);
    }


    // Small utility methods.

    /**
     * Marks the given VisitorAccepter as being used (or useful).
     * In this context, the VisitorAccepter will be an Attribute object.
     */
    private static void markAsUsed(VisitorAccepter visitorAccepter)
    {
        visitorAccepter.setVisitorInfo(USED);
    }


    /**
     * Returns whether the given VisitorAccepter has been marked as being used.
     * In this context, the VisitorAccepter will be an Attribute object.
     */
    static boolean isUsed(VisitorAccepter visitorAccepter)
    {
        return visitorAccepter.getVisitorInfo() == USED;
    }
}
