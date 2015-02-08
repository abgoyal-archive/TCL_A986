
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.AttributeVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.*;

public class VariableSizeUpdater
extends      SimplifiedVisitor
implements   AttributeVisitor,
             InstructionVisitor
{
    //*
    private static final boolean DEBUG = false;
    /*/
    private static       boolean DEBUG = true;
    //*/


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
//        DEBUG =
//            clazz.getName().equals("abc/Def") &&
//            method.getName(clazz).equals("abc");

        // The minimum variable size is determined by the arguments.
        codeAttribute.u2maxLocals =
            ClassUtil.internalMethodParameterSize(method.getDescriptor(clazz),
                                                  method.getAccessFlags());

        if (DEBUG)
        {
            System.out.println("VariableSizeUpdater: "+clazz.getName()+"."+method.getName(clazz)+method.getDescriptor(clazz));
            System.out.println("  Max locals: "+codeAttribute.u2maxLocals+" <- parameters");
        }

        // Go over all instructions.
        codeAttribute.instructionsAccept(clazz, method, this);
    }


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitVariableInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, VariableInstruction variableInstruction)
    {
        int variableSize = variableInstruction.variableIndex + 1;
        if (variableInstruction.isCategory2())
        {
            variableSize++;
        }

        if (codeAttribute.u2maxLocals < variableSize)
        {
            codeAttribute.u2maxLocals = variableSize;

            if (DEBUG)
            {
                System.out.println("Max locals: "+codeAttribute.u2maxLocals+" <- "+variableInstruction.toString(offset));
            }
        }
    }
}
