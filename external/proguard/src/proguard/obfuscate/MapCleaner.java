
package proguard.obfuscate;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.ClassVisitor;

import java.util.Map;

public class MapCleaner
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private final Map map;


    /**
     * Creates a new MapCleaner.
     * @param map the map to be cleared.
     */
    public MapCleaner(Map map)
    {
        this.map = map;
    }


    // Implementations for ClassVisitor.

    public void visitAnyClass(Clazz clazz)
    {
        map.clear();
    }
}
