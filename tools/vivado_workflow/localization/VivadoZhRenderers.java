import java.awt.Component;
import java.awt.Container;
import java.lang.ref.WeakReference;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.function.UnaryOperator;
import javax.swing.DefaultListCellRenderer;
import javax.swing.JComboBox;
import javax.swing.JList;
import javax.swing.JTable;
import javax.swing.JTree;
import javax.swing.ListCellRenderer;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.TableCellRenderer;
import javax.swing.table.TableColumn;
import javax.swing.tree.DefaultTreeCellRenderer;
import javax.swing.tree.TreeCellRenderer;

/** Reversible presentation adapters. Does not replace a model, value or editor. */
public final class VivadoZhRenderers {
    public enum Kind { TREE, LIST, COMBO, TABLE_HEADER, TABLE_CELL }

    /** Valid only during the synchronous translation callback; do not retain it. */
    public static final class Context {
        public final Kind kind;
        public final Component owner;
        public final Object value;
        public final int row;
        public final int column;
        private Context(Kind k, Component o, Object v, int r, int c) {
            kind=k;owner=o;value=v;row=r;column=c;
        }
    }

    private static final ThreadLocal<Context> current = new ThreadLocal<>();
    private static final Map<JTree,WeakReference<TreeWrapper>> trees = new WeakHashMap<>();
    private static final Map<JList<?>,WeakReference<ListWrapper>> lists = new WeakHashMap<>();
    private static final Map<JComboBox<?>,WeakReference<ListWrapper>> combos = new WeakHashMap<>();
    private static final Map<TableColumn,WeakReference<TableWrapper>> headers = new WeakHashMap<>();
    private static final Map<TableColumn,WeakReference<TableWrapper>> cells = new WeakHashMap<>();

    private VivadoZhRenderers() {}

    public static Context currentContext() { return current.get(); }

    /**
     * Installs on this component only. Call on the EDT, as for any Swing mutation.
     * The callback must limit translation to approved display strings. In particular,
     * TABLE_CELL and user-defined TREE/LIST/COMBO values are not inherently UI prose.
     */
    public static void install(Component component, UnaryOperator<Component> callback) {
        requireEdt();
        if(callback==null) throw new IllegalArgumentException("A translation callback is required");
        if(component instanceof JTree) installTree((JTree)component,callback);
        else if(component instanceof JComboBox<?>) installCombo((JComboBox<?>)component,callback);
        else if(component instanceof JList<?>) installList((JList<?>)component,callback);
        else if(component instanceof JTable) installTable((JTable)component,callback);
    }

    private static void requireEdt() {
        if(!SwingUtilities.isEventDispatchThread())
            throw new IllegalStateException("Renderer installation/restoration requires the Swing EDT");
    }

    private static boolean isWrapper(Object object) {
        // A previous reload should restore first. This guard also prevents accidental
        // cross-classloader wrapper nesting when an old owner has survived a reload.
        return object!=null && object.getClass().getName().startsWith("VivadoZhRenderers$")
            && object.getClass().getName().endsWith("Wrapper");
    }

    private static TreeCellRenderer treeDelegate(JTree tree) {
        try {
            Method getter=tree.getClass().getMethod("getWrappedCellRenderer");
            return (TreeCellRenderer)getter.invoke(tree);
        } catch(NoSuchMethodException absent) {
            return tree.getCellRenderer();
        } catch(IllegalAccessException | InvocationTargetException failed) {
            // Never wrap JXTree's stable delegating renderer after a failed unwrap.
            throw new IllegalStateException("Cannot unwrap tree renderer: "+tree.getClass().getName(),failed);
        }
    }

    private static ListCellRenderer<?> listDelegate(JList<?> list) {
        try {
            // JXList has the same stable outer/delegate arrangement as JXTree.
            Method getter=list.getClass().getMethod("getWrappedCellRenderer");
            return (ListCellRenderer<?>)getter.invoke(list);
        } catch(NoSuchMethodException absent) {
            return list.getCellRenderer();
        } catch(IllegalAccessException | InvocationTargetException failed) {
            throw new IllegalStateException("Cannot unwrap list renderer: "+list.getClass().getName(),failed);
        }
    }

    private static void installTree(JTree tree, UnaryOperator<Component> callback) {
        TreeCellRenderer original=treeDelegate(tree);
        if(isWrapper(original)) return;
        TreeWrapper wrapper=new TreeWrapper(original,callback);
        trees.put(tree,new WeakReference<>(wrapper));
        tree.setCellRenderer(wrapper);tree.revalidate();tree.repaint();
    }

    @SuppressWarnings({"unchecked","rawtypes"})
    private static void installList(JList<?> list, UnaryOperator<Component> callback) {
        ListCellRenderer<?> original=listDelegate(list);
        if(isWrapper(original)) return;
        ListWrapper wrapper=new ListWrapper((ListCellRenderer)original,callback,Kind.LIST,null);
        lists.put(list,new WeakReference<>(wrapper));
        ((JList)list).setCellRenderer(wrapper);list.revalidate();list.repaint();
    }

    @SuppressWarnings({"unchecked","rawtypes"})
    private static void installCombo(JComboBox<?> combo, UnaryOperator<Component> callback) {
        ListCellRenderer<?> original=combo.getRenderer();
        if(isWrapper(original)) return;
        ListWrapper wrapper=new ListWrapper((ListCellRenderer)original,callback,Kind.COMBO,combo);
        combos.put(combo,new WeakReference<>(wrapper));
        ((JComboBox)combo).setRenderer(wrapper);combo.revalidate();combo.repaint();
        // Editable combo-box editors and their document contents remain untouched.
    }

    private static void installTable(JTable table, UnaryOperator<Component> callback) {
        boolean changed=false;
        for(Enumeration<TableColumn> e=table.getColumnModel().getColumns();e.hasMoreElements();) {
            TableColumn column=e.nextElement();
            if(!isWrapper(column.getHeaderRenderer())) {
                TableWrapper wrapper=new TableWrapper(column.getHeaderRenderer(),callback,true);
                headers.put(column,new WeakReference<>(wrapper));column.setHeaderRenderer(wrapper);changed=true;
            }
            if(!isWrapper(column.getCellRenderer())) {
                TableWrapper wrapper=new TableWrapper(column.getCellRenderer(),callback,false);
                cells.put(column,new WeakReference<>(wrapper));column.setCellRenderer(wrapper);changed=true;
            }
        }
        if(changed) {
            table.revalidate();table.repaint();
            if(table.getTableHeader()!=null) table.getTableHeader().repaint();
        }
    }

    private static Component translate(Component rendered, UnaryOperator<Component> callback, Context context) {
        if(rendered==null) return null;
        Context previous=current.get();
        current.set(context);
        try {
            Set<Component> seen=Collections.newSetFromMap(new IdentityHashMap<Component,Boolean>());
            return translateTree(rendered,callback,seen);
        } finally {
            if(previous==null) current.remove(); else current.set(previous);
        }
    }

    private static Component translateTree(Component component, UnaryOperator<Component> callback, Set<Component> seen) {
        if(!seen.add(component)) return component;
        // Visit only the renderer's existing children. No owner components or model
        // objects are traversed. Child callbacks should mutate presentation in place.
        if(component instanceof Container)
            for(Component child:((Container)component).getComponents()) translateTree(child,callback,seen);
        Component translated=callback.apply(component);
        return translated==null?component:translated;
    }

    private static final class TreeWrapper implements TreeCellRenderer {
        final TreeCellRenderer original;
        final UnaryOperator<Component> callback;
        final DefaultTreeCellRenderer fallback=new DefaultTreeCellRenderer();
        TreeWrapper(TreeCellRenderer o, UnaryOperator<Component> c) {original=o;callback=c;}
        @Override public Component getTreeCellRendererComponent(JTree tree,Object value,boolean selected,
                boolean expanded,boolean leaf,int row,boolean focus) {
            TreeCellRenderer delegate=original==null||isWrapper(original)?fallback:original;
            Component rendered=delegate.getTreeCellRendererComponent(tree,value,selected,expanded,leaf,row,focus);
            return translate(rendered,callback,new Context(Kind.TREE,tree,value,row,-1));
        }
    }

    private static final class ListWrapper implements ListCellRenderer<Object> {
        final ListCellRenderer<Object> original;
        final UnaryOperator<Component> callback;
        final Kind kind;
        final WeakReference<JComboBox<?>> combo;
        final DefaultListCellRenderer fallback=new DefaultListCellRenderer();
        ListWrapper(ListCellRenderer<Object> o,UnaryOperator<Component> c,Kind k,JComboBox<?> owner) {
            original=o;callback=c;kind=k;combo=new WeakReference<>(owner);
        }
        @Override public Component getListCellRendererComponent(JList<?> list,Object value,int index,
                boolean selected,boolean focus) {
            ListCellRenderer<Object> delegate=original==null||isWrapper(original)?fallback:original;
            Component rendered=delegate.getListCellRendererComponent(list,value,index,selected,focus);
            Component owner=kind==Kind.COMBO?combo.get():list;
            return translate(rendered,callback,new Context(kind,owner==null?list:owner,value,index,-1));
        }
    }

    private static final class TableWrapper implements TableCellRenderer {
        final TableCellRenderer original;
        final UnaryOperator<Component> callback;
        final boolean header;
        final DefaultTableCellRenderer fallback=new DefaultTableCellRenderer();
        TableWrapper(TableCellRenderer o,UnaryOperator<Component> c,boolean h) {original=o;callback=c;header=h;}
        @Override public Component getTableCellRendererComponent(JTable table,Object value,boolean selected,
                boolean focus,int row,int column) {
            TableCellRenderer delegate=original;
            if(delegate==null) {
                if(header && table.getTableHeader()!=null) delegate=table.getTableHeader().getDefaultRenderer();
                else if(!header && column>=0 && column<table.getColumnCount())
                    delegate=table.getDefaultRenderer(table.getColumnClass(column));
            }
            if(delegate==null||isWrapper(delegate)) delegate=fallback;
            Component rendered=delegate.getTableCellRendererComponent(table,value,selected,focus,row,column);
            return translate(rendered,callback,new Context(header?Kind.TABLE_HEADER:Kind.TABLE_CELL,table,value,row,column));
        }
    }

    /** Restores only adapters still owned by this instance; external replacements win. */
    @SuppressWarnings({"unchecked","rawtypes"})
    public static void restoreAll() {
        requireEdt();
        for(Map.Entry<JTree,WeakReference<TreeWrapper>> e:new ArrayList<>(trees.entrySet())) {
            JTree tree=e.getKey();TreeWrapper wrapper=e.getValue().get();
            if(tree!=null&&wrapper!=null&&treeDelegate(tree)==wrapper) {
                tree.setCellRenderer(wrapper.original);tree.revalidate();tree.repaint();
            }
        }
        for(Map.Entry<JList<?>,WeakReference<ListWrapper>> e:new ArrayList<>(lists.entrySet())) {
            JList list=e.getKey();ListWrapper wrapper=e.getValue().get();
            if(list!=null&&wrapper!=null&&listDelegate(list)==wrapper) {
                list.setCellRenderer(wrapper.original);list.revalidate();list.repaint();
            }
        }
        for(Map.Entry<JComboBox<?>,WeakReference<ListWrapper>> e:new ArrayList<>(combos.entrySet())) {
            JComboBox combo=e.getKey();ListWrapper wrapper=e.getValue().get();
            if(combo!=null&&wrapper!=null&&combo.getRenderer()==wrapper) {
                combo.setRenderer(wrapper.original);combo.revalidate();combo.repaint();
            }
        }
        restoreColumns(headers,true);restoreColumns(cells,false);
        trees.clear();lists.clear();combos.clear();headers.clear();cells.clear();current.remove();
    }

    private static void restoreColumns(Map<TableColumn,WeakReference<TableWrapper>> registry,boolean header) {
        for(Map.Entry<TableColumn,WeakReference<TableWrapper>> e:new ArrayList<>(registry.entrySet())) {
            TableColumn column=e.getKey();TableWrapper wrapper=e.getValue().get();
            if(column!=null&&wrapper!=null) {
                if(header&&column.getHeaderRenderer()==wrapper) column.setHeaderRenderer(wrapper.original);
                if(!header&&column.getCellRenderer()==wrapper) column.setCellRenderer(wrapper.original);
            }
        }
    }
}
