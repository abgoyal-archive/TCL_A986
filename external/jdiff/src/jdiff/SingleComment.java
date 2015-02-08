package jdiff;

import java.io.*;
import java.util.*;

class SingleComment implements Comparable {

    /** The identifier for this comment. */
    public String id_ = null;

    /** The text of this comment. */
    public String text_ = null;

    /** If false, then this comment is inactive. */
    public boolean isUsed_ = true;

    public SingleComment(String id, String text) {
        // Escape the commentID in case it contains "<" or ">"
        // characters (generics)
        id_ = id.replaceAll("<", "&lt;").replaceAll(">", "&gt;");;
        text_ = text;
    }

    /** Compare two SingleComment objects using just the id. */
    public int compareTo(Object o) {
        return id_.compareTo(((SingleComment)o).id_);
    }
}
