
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

import java.util.*;

public class AttributeSorter
extends      SimplifiedVisitor
implements   ClassVisitor, MemberVisitor, AttributeVisitor, Comparator
{
    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Sort the attributes.
        Arrays.sort(programClass.attributes, 0, programClass.u2attributesCount, this);

        // Sort the attributes of the class members.
        programClass.fieldsAccept(this);
        programClass.methodsAccept(this);
    }


    // Implementations for MemberVisitor.

    public void visitProgramMember(ProgramClass programClass, ProgramMember programMember)
    {
        // Sort the attributes.
        Arrays.sort(programMember.attributes, 0, programMember.u2attributesCount, this);

        // Sort the attributes of the attributes.
        programMember.attributesAccept(programClass, this);
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Sort the attributes.
        Arrays.sort(codeAttribute.attributes, 0, codeAttribute.u2attributesCount, this);
    }


    // Implementations for Comparator.

    public int compare(Object object1, Object object2)
    {
        Attribute attribute1 = (Attribute)object1;
        Attribute attribute2 = (Attribute)object2;

        return attribute1.u2attributeNameIndex < attribute2.u2attributeNameIndex ? -1 :
               attribute1.u2attributeNameIndex > attribute2.u2attributeNameIndex ?  1 :
                                                                                    0;
    }
}
