import java.awt.Font;
import java.awt.font.TextAttribute;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.function.UnaryOperator;
import javax.swing.JComponent;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.border.Border;
import javax.swing.border.CompoundBorder;
import javax.swing.border.TitledBorder;

/** Display-only adapter for standard and Vivado 2025.2 custom title borders. */
public final class VivadoZhBorders {
    // States contain no border/component reference, so closed dialogs remain collectible.
    private static final Map<Border,State> changed = new WeakHashMap<>();
    private static final String VIVADO_TITLE_BASE = "ui.utils.swing.a.y";

    private VivadoZhBorders() {}

    /** Translate this component's border chain; callers traverse the component tree. */
    public static void install(JComponent owner,UnaryOperator<String> translator) {
        requireEdt();
        Objects.requireNonNull(owner,"A component is required");
        Objects.requireNonNull(translator,"A translator is required");
        Set<Border> seen=Collections.newSetFromMap(new IdentityHashMap<>());
        if(visit(owner.getBorder(),owner,translator,seen)) {
            owner.revalidate();
            owner.repaint();
        }
    }

    private static boolean visit(Border border,JComponent owner,UnaryOperator<String> translator,Set<Border> seen) {
        if(border==null||!seen.add(border)) return false;
        boolean updated=false;
        if(border instanceof CompoundBorder) {
            CompoundBorder compound=(CompoundBorder)border;
            updated|=visit(compound.getOutsideBorder(),owner,translator,seen);
            updated|=visit(compound.getInsideBorder(),owner,translator,seen);
        }
        Access access=access(border);
        if(access!=null) {
            String before=access.title(border);
            String after=translator.apply(before);
            if(after!=null&&!Objects.equals(before,after)) {
                State state=changed.get(border);
                if(state==null) {
                    state=new State(access);
                    changed.put(border,state);
                }
                // If Vivado replaced the title since our last pass, retain the new
                // original for rollback instead of restoring stale application state.
                if(!state.titleChanged||!Objects.equals(before,state.lastTitle)) state.originalTitle=before;
                access.title(border,after);
                state.lastTitle=after;
                state.titleChanged=true;
                Font raw=access.font(border);
                Font fallback=raw;
                if(fallback==null) fallback=UIManager.getFont("TitledBorder.font");
                if(fallback==null) fallback=owner.getFont();
                Font display=displayFont(fallback,after);
                // Standard TitledBorder.getTitleFont() hides a nullable raw value.
                // Keep it untouched to preserve exact inheritance/rollback semantics;
                // Vivado's verified custom getter exposes the nullable raw field.
                if(access.exposesRawFont()&&display!=null&&!Objects.equals(display,fallback)) {
                    if(!state.fontChanged||!Objects.equals(raw,state.lastFont)) state.originalFont=raw;
                    access.font(border,display);
                    state.lastFont=display;
                    state.fontChanged=true;
                }
                updated=true;
            }
            updated|=visit(access.inside(border),owner,translator,seen);
        }
        return updated;
    }

    /** Restore only values still owned by this adapter. Does not keep owners alive. */
    public static void restoreAll() {
        requireEdt();
        RuntimeException failure=null;
        for(Map.Entry<Border,State> entry:new ArrayList<>(changed.entrySet())) {
            Border border=entry.getKey();
            State state=entry.getValue();
            if(border==null) continue;
            try {
                if(state.titleChanged&&Objects.equals(state.access.title(border),state.lastTitle))
                    state.access.title(border,state.originalTitle);
                if(state.fontChanged&&Objects.equals(state.access.font(border),state.lastFont))
                    state.access.font(border,state.originalFont);
            } catch(RuntimeException error) {
                if(failure==null) failure=new IllegalStateException("Could not restore a title border",error);
                else failure.addSuppressed(error);
            }
        }
        changed.clear();
        if(failure!=null) throw failure;
    }

    private static Access access(Border border) {
        if(border instanceof TitledBorder) return StandardAccess.INSTANCE;
        for(Class<?> type=border.getClass();type!=null;type=type.getSuperclass())
            if(type.getName().equals(VIVADO_TITLE_BASE)) return new VivadoAccess(type);
        return null;
    }

    private static void requireEdt() {
        if(!SwingUtilities.isEventDispatchThread())
            throw new IllegalStateException("Border translation/restoration requires the Swing EDT");
    }

    private static Font displayFont(Font original,String text) {
        if(original==null||original.canDisplayUpTo(text)<0) return original;
        Map<TextAttribute,Object> attributes=new HashMap<>();
        attributes.putAll(original.getAttributes());
        attributes.put(TextAttribute.FAMILY,"Noto Sans CJK SC");
        return new Font(attributes);
    }

    private interface Access {
        String title(Border border);
        void title(Border border,String value);
        Font font(Border border);
        void font(Border border,Font value);
        boolean exposesRawFont();
        Border inside(Border border);
    }

    private static final class StandardAccess implements Access {
        private static final StandardAccess INSTANCE=new StandardAccess();
        public String title(Border b) {return ((TitledBorder)b).getTitle();}
        public void title(Border b,String v) {((TitledBorder)b).setTitle(v);}
        public Font font(Border b) {return ((TitledBorder)b).getTitleFont();}
        public void font(Border b,Font v) {((TitledBorder)b).setTitleFont(v);}
        public boolean exposesRawFont() {return false;}
        public Border inside(Border b) {return ((TitledBorder)b).getBorder();}
    }

    private static final class VivadoAccess implements Access {
        final Method getTitle,setTitle,getFont,setFont,getBorder;
        VivadoAccess(Class<?> base) {
            try {
                getTitle=base.getMethod("getTitle");
                setTitle=base.getMethod("setTitle",String.class);
                getFont=base.getMethod("jPh");
                setFont=base.getMethod("o",Font.class);
                getBorder=base.getMethod("getBorder");
            } catch(ReflectiveOperationException error) {
                throw new IllegalStateException("Unsupported Vivado title-border API",error);
            }
        }
        public String title(Border b) {return (String)call(getTitle,b);}
        public void title(Border b,String v) {call(setTitle,b,v);}
        public Font font(Border b) {return (Font)call(getFont,b);}
        public void font(Border b,Font v) {call(setFont,b,v);}
        public boolean exposesRawFont() {return true;}
        public Border inside(Border b) {return (Border)call(getBorder,b);}
        private static Object call(Method method,Object target,Object... values) {
            try {return method.invoke(target,values);}
            catch(ReflectiveOperationException error) {
                Throwable cause=error instanceof InvocationTargetException?((InvocationTargetException)error).getCause():error;
                throw new IllegalStateException("Vivado title-border method failed: "+method.getName(),cause);
            }
        }
    }

    private static final class State {
        final Access access;
        String originalTitle,lastTitle;
        Font originalFont,lastFont;
        boolean titleChanged,fontChanged;
        State(Access access) {this.access=access;}
    }
}
