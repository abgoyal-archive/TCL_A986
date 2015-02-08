
package proguard.classfile.instruction;

public abstract class SwitchInstruction extends Instruction
{
    public int   defaultOffset;
    public int[] jumpOffsets;


    /**
     * Creates an uninitialized SwitchInstruction.
     */
    public SwitchInstruction() {}


    /**
     * Creates a new SwitchInstruction with the given arguments.
     */
    public SwitchInstruction(byte  opcode,
                             int   defaultOffset,
                             int[] jumpOffsets)
    {
        this.opcode        = opcode;
        this.defaultOffset = defaultOffset;
        this.jumpOffsets   = jumpOffsets;
    }


    /**
     * Copies the given instruction into this instruction.
     * @param switchInstruction the instruction to be copied.
     * @return this instruction.
     */
    public SwitchInstruction copy(SwitchInstruction switchInstruction)
    {
        this.opcode        = switchInstruction.opcode;
        this.defaultOffset = switchInstruction.defaultOffset;
        this.jumpOffsets   = switchInstruction.jumpOffsets;

        return this;
    }


    // Implementations for Instruction.

    public String toString(int offset)
    {
        return "["+offset+"] "+toString()+" (target="+(offset+defaultOffset)+")";
    }


    // Implementations for Object.

    public String toString()
    {
        return getName()+" ("+jumpOffsets.length+" offsets, default="+defaultOffset+")";
    }
}
