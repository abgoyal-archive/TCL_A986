
package proguard.optimize;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.MemberVisitor;
import proguard.optimize.info.ReadWriteFieldMarker;

public class WriteOnlyFieldFilter
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private final MemberVisitor writeOnlyFieldVisitor;


    /**
     * Creates a new WriteOnlyFieldFilter.
     * @param writeOnlyFieldVisitor the <code>MemberVisitor</code> to which
     *                              visits to write-only fields will be delegated.
     */
    public WriteOnlyFieldFilter(MemberVisitor writeOnlyFieldVisitor)
    {
        this.writeOnlyFieldVisitor = writeOnlyFieldVisitor;
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {

        if (ReadWriteFieldMarker.isWritten(programField) &&
            !ReadWriteFieldMarker.isRead(programField))
        {
            writeOnlyFieldVisitor.visitProgramField(programClass, programField);
        }
    }
}
