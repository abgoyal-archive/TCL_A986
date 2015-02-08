
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;

import java.io.PrintStream;


public class MappingPrinter
extends      SimplifiedVisitor
implements   ClassVisitor,
             MemberVisitor
{
    private final PrintStream ps;


    /**
     * Creates a new MappingPrinter that prints to <code>System.out</code>.
     */
    public MappingPrinter()
    {
        this(System.out);
    }


    /**
     * Creates a new MappingPrinter that prints to the given stream.
     * @param printStream the stream to which to print
     */
    public MappingPrinter(PrintStream printStream)
    {
        this.ps = printStream;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        String name    = programClass.getName();
        String newName = ClassObfuscator.newClassName(programClass);

        ps.println(ClassUtil.externalClassName(name) +
                   " -> " +
                   ClassUtil.externalClassName(newName) +
                   ":");

        // Print out the class members.
        programClass.fieldsAccept(this);
        programClass.methodsAccept(this);
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        String newName = MemberObfuscator.newMemberName(programField);
        if (newName != null)
        {
            ps.println("    " +
                       //lineNumberRange(programClass, programField) +
                       ClassUtil.externalFullFieldDescription(
                           0,
                           programField.getName(programClass),
                           programField.getDescriptor(programClass)) +
                       " -> " +
                       newName);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        // Special cases: <clinit> and <init> are always kept unchanged.
        // We can ignore them here.
        String name = programMethod.getName(programClass);
        if (name.equals(ClassConstants.INTERNAL_METHOD_NAME_CLINIT) ||
            name.equals(ClassConstants.INTERNAL_METHOD_NAME_INIT))
        {
            return;
        }

        String newName = MemberObfuscator.newMemberName(programMethod);
        if (newName != null)
        {
            ps.println("    " +
                       lineNumberRange(programClass, programMethod) +
                       ClassUtil.externalFullMethodDescription(
                           programClass.getName(),
                           0,
                           programMethod.getName(programClass),
                           programMethod.getDescriptor(programClass)) +
                       " -> " +
                       newName);
        }
    }


    // Small utility methods.

    /**
     * Returns the line number range of the given class member, followed by a
     * colon, or just an empty String if no range is available.
     */
    private static String lineNumberRange(ProgramClass programClass, ProgramMember programMember)
    {
        String range = programMember.getLineNumberRange(programClass);
        return range != null ?
            (range + ":") :
            "";
    }
}
