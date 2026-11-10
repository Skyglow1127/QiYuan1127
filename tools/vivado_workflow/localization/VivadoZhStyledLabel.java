import java.awt.Component;
import java.awt.Font;
import java.awt.font.TextAttribute;
import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import javax.swing.JLabel;

/** Safe JIDE renderer copies and coupled text/style snapshots for static headings. */
public final class VivadoZhStyledLabel {
    private VivadoZhStyledLabel() {}

    /**
     * Immutable array snapshot, retaining the original StyleRange object identities.
     * JIDE StyleRange is mutable; the helper never mutates the captured objects.
     */
    public static final class Snapshot {
        public final String text;
        private final Object[] ranges;
        private Snapshot(String text,Object[] ranges) {
            this.text=text;this.ranges=ranges.clone();
        }
        public int styleCount() {return ranges.length;}
        public Object[] styleRanges() {return ranges.clone();}
        @Override public boolean equals(Object other) {
            if(this==other) return true;
            if(!(other instanceof Snapshot)) return false;
            Snapshot snapshot=(Snapshot)other;
            return Objects.equals(text,snapshot.text)&&Arrays.equals(ranges,snapshot.ranges);
        }
        @Override public int hashCode() {return 31*Objects.hashCode(text)+Arrays.hashCode(ranges);}
    }

    public static boolean isStyledLabel(Component component) {
        if(component==null) return false;
        for(Class<?> type=component.getClass();type!=null;type=type.getSuperclass())
            if(type.getName().equals("com.jidesoft.swing.StyledLabel")) return true;
        return false;
    }

    public static Snapshot read(JLabel original) {
        requireStyled(original);
        try {
            Object[] ranges=(Object[])original.getClass().getMethod("getStyleRanges").invoke(original);
            return new Snapshot(original.getText(),ranges);
        } catch(ReflectiveOperationException error) {
            throw failure("Read styled label",error);
        }
    }

    /** Chinese text has no copied English character offsets; original ranges remain saved. */
    public static Snapshot translated(Snapshot original,String translatedText) {
        Objects.requireNonNull(original,"Original snapshot is required");
        Object[] empty=(Object[])Array.newInstance(original.ranges.getClass().getComponentType(),0);
        return new Snapshot(translatedText,empty);
    }

    /**
     * Apply on the Swing EDT. The order is essential: clear offsets, change text,
     * then install matching offsets. Component names used by automation are kept.
     * The caller must save and restore this whole snapshot as one change property.
     */
    public static void apply(JLabel target,Snapshot state) {
        requireStyled(target);Objects.requireNonNull(state,"Snapshot is required");
        final Method clear,set;
        try {
            clear=target.getClass().getMethod("clearStyleRanges");
            set=target.getClass().getMethod("setStyleRanges",state.ranges.getClass());
        } catch(ReflectiveOperationException error) {
            // Resolve both before changing anything, including classloader compatibility.
            throw failure("Resolve styled label methods",error);
        }
        String originalName=target.getName();
        try {
            clear.invoke(target);
            target.setText(state.text);
            // Pass the typed array as one reflection argument, not varargs elements.
            set.invoke(target,(Object)state.ranges.clone());
        } catch(ReflectiveOperationException error) {
            throw failure("Apply styled label snapshot",error);
        } finally {
            if(!Objects.equals(originalName,target.getName())) target.setName(originalName);
        }
    }

    private static void requireStyled(JLabel label) {
        if(!isStyledLabel(label)) throw new IllegalArgumentException("A JIDE StyledLabel is required");
    }

    private static IllegalStateException failure(String operation,ReflectiveOperationException error) {
        Throwable cause=error instanceof InvocationTargetException?((InvocationTargetException)error).getCause():error;
        return new IllegalStateException(operation,cause);
    }

    /**
     * Returns a standalone plain Swing label for renderer presentation. Never calls
     * setText, setFont, setStyleRanges, or any other setter on the original component.
     * Do not install this copy into the application's source/model component tree.
     */
    public static Component translatedCopy(JLabel original,String translatedText) {
        if(original==null) throw new IllegalArgumentException("Original label is required");
        JLabel copy=new JLabel(translatedText);
        copy.setIcon(original.getIcon());
        copy.setDisabledIcon(original.getDisabledIcon());
        copy.setFont(displayFont(original.getFont(),translatedText));
        copy.setForeground(original.getForeground());
        copy.setBackground(original.getBackground());
        copy.setBorder(original.getBorder());
        copy.setHorizontalAlignment(original.getHorizontalAlignment());
        copy.setVerticalAlignment(original.getVerticalAlignment());
        copy.setHorizontalTextPosition(original.getHorizontalTextPosition());
        copy.setVerticalTextPosition(original.getVerticalTextPosition());
        copy.setIconTextGap(original.getIconTextGap());
        copy.setEnabled(original.isEnabled());
        copy.setOpaque(original.isOpaque());
        copy.setToolTipText(original.getToolTipText());
        copy.setComponentOrientation(original.getComponentOrientation());
        copy.setLocale(original.getLocale());
        // Original mnemonic/style-range indices refer to the untranslated string.
        // They must not be copied to the shorter translated string.
        return copy;
    }

    private static Font displayFont(Font original,String text) {
        if(original==null||text==null||original.canDisplayUpTo(text)<0) return original;
        Map<TextAttribute,Object> attributes=new HashMap<>();
        attributes.putAll(original.getAttributes());
        attributes.put(TextAttribute.FAMILY,"Noto Sans CJK SC");
        return new Font(attributes);
    }
}
