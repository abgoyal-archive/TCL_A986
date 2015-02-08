
package proguard.optimize.info;

import proguard.classfile.*;
import proguard.classfile.attribute.CodeAttribute;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.instruction.*;
import proguard.classfile.instruction.visitor.InstructionVisitor;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class SideEffectInstructionChecker
extends      SimplifiedVisitor
implements   InstructionVisitor,
             ConstantVisitor,
             MemberVisitor
{
    private final boolean includeReturnInstructions;

    // A return value for the visitor methods.
    private boolean hasSideEffects;


    public SideEffectInstructionChecker(boolean includeReturnInstructions)
    {
        this.includeReturnInstructions = includeReturnInstructions;
    }


    public boolean hasSideEffects(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction)
    {
        hasSideEffects = false;

        instruction.accept(clazz, method,  codeAttribute, offset, this);

        return hasSideEffects;
    }


    // Implementations for InstructionVisitor.

    public void visitAnyInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, Instruction instruction) {}


    public void visitSimpleInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, SimpleInstruction simpleInstruction)
    {
        byte opcode = simpleInstruction.opcode;

        // Check for instructions that might cause side effects.
        if (opcode == InstructionConstants.OP_IASTORE      ||
            opcode == InstructionConstants.OP_LASTORE      ||
            opcode == InstructionConstants.OP_FASTORE      ||
            opcode == InstructionConstants.OP_DASTORE      ||
            opcode == InstructionConstants.OP_AASTORE      ||
            opcode == InstructionConstants.OP_BASTORE      ||
            opcode == InstructionConstants.OP_CASTORE      ||
            opcode == InstructionConstants.OP_SASTORE      ||
            opcode == InstructionConstants.OP_ATHROW       ||
            opcode == InstructionConstants.OP_MONITORENTER ||
            opcode == InstructionConstants.OP_MONITOREXIT  ||
            (includeReturnInstructions &&
             (opcode == InstructionConstants.OP_IRETURN ||
              opcode == InstructionConstants.OP_LRETURN ||
              opcode == InstructionConstants.OP_FRETURN ||
              opcode == InstructionConstants.OP_DRETURN ||
              opcode == InstructionConstants.OP_ARETURN ||
              opcode == InstructionConstants.OP_RETURN)))
        {
            // These instructions always cause a side effect.
            hasSideEffects = true;
        }

    }


    public void visitVariableInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, VariableInstruction variableInstruction)
    {
        byte opcode = variableInstruction.opcode;

        // Check for instructions that might cause side effects.
        if (includeReturnInstructions &&
            opcode == InstructionConstants.OP_RET)
        {
            hasSideEffects = true;
        }
    }


    public void visitConstantInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, ConstantInstruction constantInstruction)
    {
        byte opcode = constantInstruction.opcode;

        // Check for instructions that might cause side effects.
        if (opcode == InstructionConstants.OP_PUTSTATIC     ||
            opcode == InstructionConstants.OP_PUTFIELD      ||
            opcode == InstructionConstants.OP_INVOKEVIRTUAL ||
            opcode == InstructionConstants.OP_INVOKESPECIAL ||
            opcode == InstructionConstants.OP_INVOKESTATIC  ||
            opcode == InstructionConstants.OP_INVOKEINTERFACE)
        {
            // Check if the field is write-only or volatile, or if the invoked
            // method is causing any side effects.
            clazz.constantPoolEntryAccept(constantInstruction.constantIndex, this);
        }
    }


    public void visitBranchInstruction(Clazz clazz, Method method, CodeAttribute codeAttribute, int offset, BranchInstruction branchInstruction)
    {
        byte opcode = branchInstruction.opcode;

        // Check for instructions that might cause side effects.
        if (includeReturnInstructions &&
            (opcode == InstructionConstants.OP_JSR ||
             opcode == InstructionConstants.OP_JSR_W))
        {
            hasSideEffects = true;
        }
    }


    // Implementations for ConstantVisitor.

    public void visitFieldrefConstant(Clazz clazz, FieldrefConstant fieldrefConstant)
    {
        // We'll have to assume accessing an unknown field has side effects.
        hasSideEffects = true;

        // Check the referenced field.
        fieldrefConstant.referencedMemberAccept(this);
    }


    public void visitAnyMethodrefConstant(Clazz clazz, RefConstant refConstant)
    {
        Member referencedMember = refConstant.referencedMember;

        // Do we have a reference to the method?
        if (referencedMember == null)
        {
            // We'll have to assume invoking the unknown method has side effects.
            hasSideEffects = true;
        }
        else
        {
            // First check the referenced method itself.
            refConstant.referencedMemberAccept(this);

            // If the result isn't conclusive, check down the hierarchy.
            if (!hasSideEffects)
            {
                Clazz  referencedClass  = refConstant.referencedClass;
                Method referencedMethod = (Method)referencedMember;

                // Check all other implementations of the method down the class
                // hierarchy.
                if ((referencedMethod.getAccessFlags() & ClassConstants.INTERNAL_ACC_PRIVATE) == 0)
                {
                    clazz.hierarchyAccept(false, false, false, true,
                                          new NamedMethodVisitor(referencedMethod.getName(referencedClass),
                                                                 referencedMethod.getDescriptor(referencedClass),
                                          this));
                }
            }
        }
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        hasSideEffects = ReadWriteFieldMarker.isRead(programField);
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        hasSideEffects = hasSideEffects ||
                         SideEffectMethodMarker.hasSideEffects(programMethod);
    }


    public void visitLibraryField(LibraryClass libraryClass, LibraryField libraryField)
    {
        hasSideEffects = true;
    }


    public void visitLibraryMethod(LibraryClass libraryClass, LibraryMethod libraryMethod)
    {
        hasSideEffects = hasSideEffects ||
                         !NoSideEffectMethodMarker.hasNoSideEffects(libraryMethod);
    }
}
