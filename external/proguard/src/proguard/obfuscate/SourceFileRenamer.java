
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.editor.ConstantPoolEditor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

public class SourceFileRenamer
extends      SimplifiedVisitor
implements   ClassVisitor,
             AttributeVisitor
{
    private final String newSourceFileAttribute;


    /**
     * Creates a new SourceFileRenamer.
     * @param newSourceFileAttribute the new string to be put in the source file
     *                               attributes.
     */
    public SourceFileRenamer(String newSourceFileAttribute)
    {
        this.newSourceFileAttribute = newSourceFileAttribute;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        // Only visit the class attributes.
        programClass.attributesAccept(this);
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitSourceFileAttribute(Clazz clazz, SourceFileAttribute sourceFileAttribute)
    {
        // Fix the source file attribute.
        sourceFileAttribute.u2sourceFileIndex =
            new ConstantPoolEditor((ProgramClass)clazz).addUtf8Constant(newSourceFileAttribute);
    }


    public void visitSourceDirAttribute(Clazz clazz, SourceDirAttribute sourceDirAttribute)
    {
        // Fix the source file attribute.
        sourceDirAttribute.u2sourceDirIndex =
            new ConstantPoolEditor((ProgramClass)clazz).addUtf8Constant(newSourceFileAttribute);
    }
}
