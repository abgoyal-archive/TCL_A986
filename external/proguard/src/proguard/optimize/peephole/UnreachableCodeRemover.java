
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.editor.CodeAttributeEditor;
import proguard.classfile.instruction.Instruction;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class UnreachableCodeRemover
extends      SimplifiedVisitor
implements   AttributeVisitor,
             InstructionVisitor
{
    //*
    private static final boolean DEBUG = false;
    /*/
    private static       boolean DEBUG = true;
    //*/

    private final InstructionVisitor  extraInstructionVisitor;

    private final ReachableCodeMarker reachableCodeMarker = new ReachableCodeMarker();
    private final CodeAttributeEditor codeAttributeEditor = new CodeAttributeEditor();


    /**
     * Creates a new UnreachableCodeRemover.
     */
    public UnreachableCodeRemover()
    {
        this(null);
    }


    /**
     * Creates a new UnreachableCodeRemover.
     * @param extraInstructionVisitor an optional extra visitor for all
     *                                deleted instructions.
     */
    public UnreachableCodeRemover(InstructionVisitor  extraInstructionVisitor)
    {
        this.extraInstructionVisitor = extraInstructionVisitor;
    }


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
//        DEBUG =
//            clazz.getName().equals("abc/Def") &&
//            method.getName(clazz).equals("abc");

        // TODO: Remove this when the code has stabilized.
        // Catch any unexpected exceptions from the actual visiting method.
        try
        {
            // Process the code.
            visitCodeAttribute0(clazz, method, codeAttribute);
        }
        catch (RuntimeException ex)
        {
            System.err.println("Unexpected error while removing unreachable code:");
            System.err.println("  Class       = ["+clazz.getName()+"]");
            System.err.println("  Method      = ["+method.getName(clazz)+method.getDescriptor(clazz)+"]");
            System.err.println("  Exception   = ["+ex.getClass().getName()+"] ("+ex.getMessage()+")");

            throw ex;
        }
    }


    public void visitCodeAttribute0(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        if (DEBUG)
        {
            System.out.println("UnreachableCodeRemover: "+clazz.getName()+"."+method.getName(clazz)+method.getDescriptor(clazz));
        }

        reachableCodeMarker.visitCodeAttribute(clazz, method, codeAttribute);

        codeAttributeEditor.reset(codeAttribute.u4codeLength);

        codeAttribute.instructionsAccept(clazz, method, this);

        codeAttributeEditor.visitCodeAttribute(clazz, method, codeAttribute);
    }


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction)
    {
        if (DEBUG)
        {
            System.out.println("  "+(reachableCodeMarker.isReachable(offset) ? "+" : "-")+" "+instruction.toString(offset));
        }

        // Is this instruction unreachable?
        if (!reachableCodeMarker.isReachable(offset))
        {
            // Then delete it.
            codeAttributeEditor.deleteInstruction(offset);

            // Visit the instruction, if required.
            if (extraInstructionVisitor != null)
            {
                instruction.accept(clazz, method, codeAttribute, offset, extraInstructionVisitor);
            }
        }
    }
}
