import java.awt.Component;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.function.UnaryOperator;
import javax.swing.JLabel;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.TableCellRenderer;

/**
 * Display-label discovery for Vivado 2025.2's native IP parameter tables.
 *
 * These tables override getCellRenderer and bypass TableColumn renderers. Their
 * update path casts fDi elements back to the concrete bc class, so wrapping or
 * replacing those elements would break parameter refresh. Only the existing
 * bc.fEg display label is offered to the caller. No renderer, editor, collection,
 * native model, parameter value or object identity is changed here.
 */
public final class VivadoZhIpTables {
    private static final String TABLE="ui.data.coregen.xpegua.aD";
    private static final String TEXT_RENDERER="ui.data.coregen.xpegua.bc";
    private static final String TEXT_LABEL="ui.data.coregen.xpegua.ba";
    // Clocking Wizard uses the parallel XPG implementation, not HACGC.
    private static final String XPG_TABLE="ui.data.coregen.xpegua.cC";
    private static final String XPG_RENDERER="ui.data.coregen.xpegua.cK";
    private static final String XPG_LABEL="ui.data.coregen.xpegua.cJ";
    private static final Set<JLabel> observedLabels=
        Collections.newSetFromMap(new WeakHashMap<JLabel,Boolean>());

    private static final ClassValue<Field> renderersField=new ClassValue<Field>() {
        @Override protected Field computeValue(Class<?> type) {
            Field field=field(type,"fDi");
            if(field.getType()!=ArrayList.class)
                throw new IllegalStateException("Unsupported Vivado IP table renderer collection: "+field);
            return field;
        }
    };
    private static final ClassValue<Field> labelField=new ClassValue<Field>() {
        @Override protected Field computeValue(Class<?> type) {
            boolean xpg=type.getName().equals(XPG_RENDERER);
            Field field=field(type,xpg?"fGq":"fEg");
            if(!field.getType().getName().equals(xpg?XPG_LABEL:TEXT_LABEL))
                throw new IllegalStateException("Unsupported Vivado IP text renderer label: "+field);
            return field;
        }
    };

    private VivadoZhIpTables() {}

    /**
     * Invoke on the EDT during the existing UI scan. The callback must translate
     * in place and return the exact same component. The caller owns text/style/
     * font snapshots and rollback. Repeated scans also discover refreshed lists
     * and externally replaced labels without retaining closed IP windows.
     */
    public static void install(JTable table,UnaryOperator<Component> callback) {
        requireEdt();
        if(table==null||callback==null) throw new IllegalArgumentException("Table and in-place callback are required");
        boolean xpg=table.getClass().getName().equals(XPG_TABLE);
        if(!xpg && !table.getClass().getName().equals(TABLE)) return;
        Object value=read(renderersField.get(table.getClass()),table);
        if(value==null) return; // Table construction/refresh may not have completed.
        if(!(value instanceof ArrayList<?>))
            throw new IllegalStateException("Unsupported Vivado IP table renderer collection instance");
        for(Object renderer:new ArrayList<>((ArrayList<?>)value)) {
            if(renderer==null||!renderer.getClass().getName().equals(xpg?XPG_RENDERER:TEXT_RENDERER)) continue;
            if(!(renderer instanceof TableCellRenderer))
                throw new IllegalStateException("Vivado IP text renderer no longer implements TableCellRenderer");
            Object label=read(labelField.get(renderer.getClass()),renderer);
            if(label==null) continue;
            if(!(label instanceof JLabel)||!label.getClass().getName().equals(xpg?XPG_LABEL:TEXT_LABEL))
                throw new IllegalStateException("Unsupported Vivado IP text renderer label instance");
            Component original=(Component)label;
            if(callback.apply(original)!=original)
                throw new IllegalStateException("Vivado IP text callback must return the same label; renderer replacement is unsafe");
            observedLabels.add((JLabel)label);
        }
    }

    /** This helper changes no properties; the caller restores its label snapshots. */
    public static void restoreAll() {
        requireEdt();observedLabels.clear();
    }

    private static void requireEdt() {
        if(!SwingUtilities.isEventDispatchThread())
            throw new IllegalStateException("Vivado IP label discovery/restoration requires the Swing EDT");
    }
    private static Field field(Class<?> type,String name) {
        try {
            Field field=type.getDeclaredField(name);
            field.setAccessible(true);
            return field;
        } catch(ReflectiveOperationException|RuntimeException error) {
            throw new IllegalStateException("Cannot inspect Vivado 2025.2 IP display field "+type.getName()+"."+name,error);
        }
    }
    private static Object read(Field field,Object target) {
        try {return field.get(target);}
        catch(IllegalAccessException error) {
            throw new IllegalStateException("Cannot read Vivado IP display field "+field,error);
        }
    }
}
