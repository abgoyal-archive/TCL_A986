
package proguard.gui;

import javax.swing.*;
import java.io.*;
import java.lang.reflect.InvocationTargetException;


final class TextAreaOutputStream extends FilterOutputStream implements Runnable
{
    private final JTextArea textArea;


    public TextAreaOutputStream(JTextArea textArea)
    {
        super(new ByteArrayOutputStream());

        this.textArea = textArea;
    }


    // Implementation for FilterOutputStream.

    public void flush() throws IOException
    {
        super.flush();

        try
        {
            // Append the accumulated buffer contents to the text area.
            SwingUtil.invokeAndWait(this);
        }
        catch (Exception e)
        {
            // Nothing.
        }
    }


    // Implementation for Runnable.

    public void run()
    {
        ByteArrayOutputStream out = (ByteArrayOutputStream)super.out;

        // Has any new text been written?
        String text = out.toString();
        if (text.length() > 0)
        {
            // Append the accumulated text to the text area.
            textArea.append(text);

            // Clear the buffer.
            out.reset();
        }
    }
}
