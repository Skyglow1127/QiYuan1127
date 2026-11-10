import java.awt.Component;
import java.io.*;
import java.util.ArrayList;
import java.util.EventListener;
import java.util.List;
import java.util.function.UnaryOperator;
import javax.swing.SwingUtilities;
import javax.swing.event.DocumentListener;
import javax.swing.event.UndoableEditListener;
import javax.swing.text.*;
import javax.swing.text.html.HTML;
import javax.swing.text.html.HTMLDocument;

/**
 * Translates a separate HTMLDocument without an HTMLWriter/reparse round trip.
 * Swing's HTMLWriter changes CSS font-size:12 into HTML font size="12", which
 * means something different when parsed. A bounded in-memory structural copy
 * preserves the original CSS values, element attributes and independent styles.
 */
public final class VivadoZhHtml {
    private static final int MAX_TEXT=200_000, MAX_BYTES=4*1024*1024;
    private static final int MAX_DEPTH=64, MAX_OBJECTS=50_000, MAX_ARRAY=200_000;
    private VivadoZhHtml() {}

    private static final class Replacement {
        final int offset,length;
        final String text;
        Replacement(int start,int size,String value) {offset=start;length=size;text=value;}
    }

    /**
     * Returns null when no approved leaf text changes. Failures throw rather than
     * risking an altered source; callers should retain the original English pane.
     * Translator receives each plain text leaf independently, never markup,
     * hyperlink destinations, code/preformatted text or input/form values.
     */
    public static HTMLDocument translatedCopy(HTMLDocument source,UnaryOperator<String> translator)
            throws IOException,BadLocationException {
        if(!SwingUtilities.isEventDispatchThread())
            throw new IllegalStateException("HTML presentation translation requires the Swing EDT");
        if(source==null || translator==null) throw new IllegalArgumentException("Source and translator required");
        if(source.getClass()!=HTMLDocument.class)
            throw new IOException("Unsupported HTMLDocument subclass: "+source.getClass().getName());
        if(source.getLength()>MAX_TEXT) throw new IOException("HTML presentation text exceeds copy limit");
        List<Replacement> replacements=new ArrayList<>();
        long projectedLength=source.getLength();
        ElementIterator iterator=new ElementIterator(source);
        for(Element element=iterator.next();element!=null;element=iterator.next()) {
            if(iterator.depth()>MAX_DEPTH) throw new IOException("HTML element nesting exceeds copy limit");
            if(!element.isLeaf() || !isProse(element)) continue;
            int offset=element.getStartOffset(),end=Math.min(element.getEndOffset(),source.getLength());
            if(end<=offset) continue;
            String text=source.getText(offset,end-offset);
            int first=0,last=text.length();
            while(first<last&&space(text.charAt(first))) first++;
            while(last>first&&space(text.charAt(last-1))) last--;
            if(first==last) continue;
            String core=text.substring(first,last),translated=translator.apply(core);
            if(translated==null || translated.equals(core)) continue;
            // Translation is not allowed to add/remove paragraph structure or
            // special embedded-object markers. Styled leaf boundaries are kept.
            if(translated.length()>MAX_TEXT || structural(translated)!=0 || structural(core)!=0
                    || translated.indexOf('\uFFFC')>=0 || translated.indexOf('\u0000')>=0) continue;
            projectedLength+=translated.length()-core.length();
            if(projectedLength>MAX_TEXT) throw new IOException("Translated HTML text exceeds copy limit");
            replacements.add(new Replacement(offset+first,last-first,translated));
        }
        if(replacements.isEmpty()) return null;
        HTMLDocument copy=copy(source);
        // Reverse order preserves all original offsets. Use attributes from the
        // copy, so no copied leaf acquires a reference to the source stylesheet.
        for(int i=replacements.size()-1;i>=0;i--) {
            Replacement replacement=replacements.get(i);
            AttributeSet attributes=copy.getCharacterElement(replacement.offset).getAttributes().copyAttributes();
            copy.replace(replacement.offset,replacement.length,replacement.text,attributes);
        }
        return copy;
    }

    private static boolean space(char c) {return Character.isWhitespace(c)||Character.isSpaceChar(c);}
    private static int structural(String text) {
        // Leaf content should be single-line; reject internal line breaks rather
        // than joining independently styled sentences or changing HTML structure.
        for(int i=0;i<text.length();i++) if(text.charAt(i)=='\n'||text.charAt(i)=='\r') return 1;
        return 0;
    }

    private static boolean blockedTag(Object name) {
        return name==HTML.Tag.HEAD || name==HTML.Tag.STYLE || name==HTML.Tag.SCRIPT
            || name==HTML.Tag.CODE || name==HTML.Tag.PRE || name==HTML.Tag.TEXTAREA
            || name==HTML.Tag.INPUT || name==HTML.Tag.SELECT || name==HTML.Tag.OPTION
            || name==HTML.Tag.OBJECT || name==HTML.Tag.APPLET;
    }

    private static boolean isProse(Element leaf) {
        if(leaf.getAttributes().getAttribute(StyleConstants.NameAttribute)!=HTML.Tag.CONTENT) return false;
        for(Element element=leaf;element!=null;element=element.getParentElement()) {
            AttributeSet attributes=element.getAttributes();
            if(blockedTag(attributes.getAttribute(StyleConstants.NameAttribute))) return false;
            for(java.util.Enumeration<?> names=attributes.getAttributeNames();names.hasMoreElements();)
                if(blockedTag(names.nextElement())) return false;
        }
        return true;
    }

    private static HTMLDocument copy(HTMLDocument source) throws IOException {
        BoundedOutput bytes=new BoundedOutput();
        try(ObjectOutputStream stream=new ObjectOutputStream(bytes) {
            private int objects;
            {enableReplaceObject(true);}
            @Override protected Object replaceObject(Object object) throws IOException {
                // Exclude listeners BEFORE traversing their object graphs. Never
                // detach anything from the live source document to serialize it.
                if(object instanceof EventListener) return null;
                if(object!=null && (++objects>MAX_OBJECTS || !allowed(object.getClass())))
                    throw new IOException("Unsupported/oversized HTML object graph: "+object.getClass().getName());
                if(object instanceof Component)
                    throw new IOException("UI components are not part of an HTML presentation copy");
                return object;
            }
        }) {stream.writeObject(source);}
        try(ObjectInputStream stream=new ObjectInputStream(new ByteArrayInputStream(bytes.toByteArray()))) {
            stream.setObjectInputFilter(info -> {
                if(info.depth()>MAX_DEPTH || info.references()>MAX_OBJECTS || info.streamBytes()>MAX_BYTES
                        || info.arrayLength()>MAX_ARRAY) return ObjectInputFilter.Status.REJECTED;
                Class<?> type=info.serialClass();
                return type==null?ObjectInputFilter.Status.UNDECIDED:
                    (allowed(type)?ObjectInputFilter.Status.ALLOWED:ObjectInputFilter.Status.REJECTED);
            });
            Object object=stream.readObject();
            if(object.getClass()!=HTMLDocument.class) throw new IOException("Unexpected presentation document type");
            HTMLDocument document=(HTMLDocument)object;
            // Defense in depth: no external callback may fire as we edit the copy.
            for(DocumentListener listener:document.getDocumentListeners()) document.removeDocumentListener(listener);
            for(UndoableEditListener listener:document.getUndoableEditListeners()) document.removeUndoableEditListener(listener);
            return document;
        } catch(ClassNotFoundException failure) {throw new IOException("Cannot copy HTML presentation",failure);}
    }

    private static boolean allowed(Class<?> type) {
        while(type.isArray()) type=type.getComponentType();
        if(type.isPrimitive()) return true;
        if(Component.class.isAssignableFrom(type)||EventListener.class.isAssignableFrom(type)) return false;
        String name=type.getName();
        return name.startsWith("javax.swing.text.") || name.equals("javax.swing.event.EventListenerList")
            || name.startsWith("java.util.") || name.startsWith("java.lang.")
            || name.equals("java.net.URL") || name.equals("java.awt.Color") || name.equals("java.awt.Font")
            || name.startsWith("java.awt.font.") || name.startsWith("java.text.AttributedCharacterIterator$")
            || name.equals("javax.swing.plaf.ColorUIResource") || name.equals("javax.swing.plaf.FontUIResource");
    }

    private static final class BoundedOutput extends OutputStream {
        private final ByteArrayOutputStream bytes=new ByteArrayOutputStream();
        @Override public void write(int value) throws IOException {
            if(bytes.size()>=MAX_BYTES) throw new IOException("HTML presentation copy exceeds byte limit");
            bytes.write(value);
        }
        @Override public void write(byte[] value,int offset,int length) throws IOException {
            if(length>MAX_BYTES-bytes.size()) throw new IOException("HTML presentation copy exceeds byte limit");
            bytes.write(value,offset,length);
        }
        byte[] toByteArray() {return bytes.toByteArray();}
    }
}
