
package proguard.classfile.attribute.visitor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class AllAttributeVisitor
extends      SimplifiedVisitor
implements   ClassVisitor,
             MemberVisitor,
             AttributeVisitor
{
    private final boolean          deep;
    private final AttributeVisitor attributeVisitor;


    /**
     * Creates a new shallow AllAttributeVisitor.
     * @param attributeVisitor the AttributeVisitor to which visits will be
     *                         delegated.
     */
    public AllAttributeVisitor(AttributeVisitor attributeVisitor)
    {
        this(false, attributeVisitor);
    }


    /**
     * Creates a new optionally deep AllAttributeVisitor.
     * @param deep             specifies whether the attributes contained
     *                         further down the class structure should be
     *                         visited too.
     * @param attributeVisitor the AttributeVisitor to which visits will be
     *                         delegated.
     */
    public AllAttributeVisitor(boolean          deep,
                               AttributeVisitor attributeVisitor)
    {
        this.deep             = deep;
        this.attributeVisitor = attributeVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        programClass.attributesAccept(attributeVisitor);

        // Visit the attributes further down the class structure, if required.
        if (deep)
        {
            programClass.fieldsAccept(this);
            programClass.methodsAccept(this);
            programClass.attributesAccept(this);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass) {}


    // Implementations for MemberVisitor.

    public void visitProgramMember(ProgramClass programClass, ProgramMember programMember)
    {
        programMember.attributesAccept(programClass, attributeVisitor);

        // Visit the attributes further down the member structure, if required.
        if (deep)
        {
            programMember.attributesAccept(programClass, this);
        }
    }


    public void visitLibraryMember(LibraryClass programClass, LibraryMember programMember) {}


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        codeAttribute.attributesAccept(clazz, method, attributeVisitor);
    }
}
