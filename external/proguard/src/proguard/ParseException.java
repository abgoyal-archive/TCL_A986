
package proguard;


public class ParseException extends Exception {

    /**
     * Constructs a <code>ParseException</code> with <code>null</code>
     * as its error detail message.
     */
    public ParseException() {
        super();
    }

    /**
     * Constructs a <code>ParseException</code> with the specified detail
     * message. The error message string <code>s</code> can later be
     * retrieved by the <code>{@link Throwable#getMessage}</code>
     * method of class <code>Throwable</code>.
     *
     * @param s the detail message.
     */
    public ParseException(String s) {
        super(s);
    }
}
