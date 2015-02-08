
package proguard.io;

import java.io.IOException;

public interface Finisher
{
    /**
     * Finishes an output stream right before it is closed.
     */
    public void finish() throws IOException;
}
