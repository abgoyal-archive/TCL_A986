
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

import java.util.Arrays;

public class InterfaceSorter
extends      SimplifiedVisitor
implements   ClassVisitor
{
    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        int[] interfaces      = programClass.u2interfaces;
        int   interfacesCount = programClass.u2interfacesCount;

        // Sort the interfaces.
        Arrays.sort(interfaces, 0, interfacesCount);

        // Remove any duplicate entries.
        int newInterfacesCount     = 0;
        int previousInterfaceIndex = 0;
        for (int index = 0; index < interfacesCount; index++)
        {
            int interfaceIndex = interfaces[index];

            // Isn't this a duplicate of the previous interface?
            if (interfaceIndex != previousInterfaceIndex)
            {
                interfaces[newInterfacesCount++] = interfaceIndex;

                // Remember the interface.
                previousInterfaceIndex = interfaceIndex;
            }
        }

        programClass.u2interfacesCount = newInterfacesCount;
    }
}
