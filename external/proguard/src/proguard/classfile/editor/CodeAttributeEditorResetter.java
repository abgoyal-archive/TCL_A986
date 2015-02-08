
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class CodeAttributeEditorResetter
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    private final CodeAttributeEditor codeAttributeEditor;


    /**
     * Creates a new CodeAttributeEditorResetter.
     * @param codeAttributeEditor the code attribute editor that will be reset.
     */
    public CodeAttributeEditorResetter(CodeAttributeEditor codeAttributeEditor)
    {
        this.codeAttributeEditor = codeAttributeEditor;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        codeAttributeEditor.reset(codeAttribute.u4codeLength);
    }
}
