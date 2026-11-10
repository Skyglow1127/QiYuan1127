import java.awt.Component;
import java.lang.ref.WeakReference;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.UnaryOperator;
import javax.swing.*;
import javax.swing.table.*;
import javax.swing.tree.*;

/** Run with -Djava.awt.headless=true; no Vivado or proprietary classes required. */
public final class VivadoZhRenderersTest {
    private static int checks;
    private static void check(boolean condition,String description) {
        checks++;if(!condition) throw new AssertionError(description);
    }
    private static final UnaryOperator<Component> TRANSLATE=component -> {
        check(VivadoZhRenderers.currentContext()!=null,"context supplied during callback");
        if(component instanceof JLabel) {
            JLabel label=(JLabel)component;
            if("General".equals(label.getText())) label.setText("常规");
            if("Name".equals(label.getText())) label.setText("名称");
        }
        return component;
    };

    /** Behaves like JXTree: stable outer renderer delegates through a separate getter. */
    public static final class DelegatingTree extends JTree {
        private TreeCellRenderer delegate;
        private final TreeCellRenderer outer=(tree,value,selected,expanded,leaf,row,focus) ->
            delegate.getTreeCellRendererComponent(tree,value,selected,expanded,leaf,row,focus);
        public DelegatingTree() {delegate=new DefaultTreeCellRenderer();super.setCellRenderer(outer);}
        public TreeCellRenderer getWrappedCellRenderer() {return delegate;}
        @Override public void setCellRenderer(TreeCellRenderer value) {
            if(outer==null) {super.setCellRenderer(value);return;}
            delegate=value;super.setCellRenderer(outer);
        }
    }

    public static final class DelegatingList extends JList<Object> {
        private ListCellRenderer<Object> delegate;
        private final ListCellRenderer<Object> outer=(list,value,index,selected,focus) ->
            delegate.getListCellRendererComponent(list,value,index,selected,focus);
        public DelegatingList() {delegate=new DefaultListCellRenderer();super.setCellRenderer(outer);}
        public ListCellRenderer<Object> getWrappedCellRenderer() {return delegate;}
        @SuppressWarnings("unchecked")
        @Override public void setCellRenderer(ListCellRenderer<? super Object> value) {
            if(outer==null) {super.setCellRenderer(value);return;}
            delegate=(ListCellRenderer<Object>)value;super.setCellRenderer(outer);
        }
    }

    private static void trees() {
        DefaultMutableTreeNode node=new DefaultMutableTreeNode("General");
        JTree tree=new JTree(node);tree.setSelectionRow(0);
        TreeModel model=tree.getModel();TreeCellRenderer original=tree.getCellRenderer();
        VivadoZhRenderers.install(tree,TRANSLATE);
        TreeCellRenderer wrapper=tree.getCellRenderer();
        VivadoZhRenderers.install(tree,TRANSLATE);
        check(wrapper==tree.getCellRenderer(),"tree installation idempotent");
        Component rendered=wrapper.getTreeCellRendererComponent(tree,node,true,true,true,0,false);
        check("常规".equals(((JLabel)rendered).getText()),"tree display translated");
        check(tree.getModel()==model&&"General".equals(node.getUserObject()),"tree model/value preserved");
        check(tree.getSelectionRows()[0]==0,"tree selection preserved");
        VivadoZhRenderers.restoreAll();check(tree.getCellRenderer()==original,"tree delegate restored");

        DelegatingTree jx=new DelegatingTree();TreeCellRenderer stable=jx.getCellRenderer();
        TreeCellRenderer inner=jx.getWrappedCellRenderer();
        VivadoZhRenderers.install(jx,TRANSLATE);
        check(jx.getCellRenderer()==stable&&jx.getWrappedCellRenderer()!=inner,"JXTree wraps inner renderer only");
        rendered=jx.getCellRenderer().getTreeCellRendererComponent(jx,node,true,true,true,0,false);
        check("常规".equals(((JLabel)rendered).getText()),"JXTree rendering has no recursion");
        VivadoZhRenderers.restoreAll();check(jx.getWrappedCellRenderer()==inner,"JXTree inner restored");
        // BasicTreeUI eagerly replaces null with a default; detach it to exercise
        // the legitimate null property path independently of that UI behavior.
        tree.setUI(null);tree.setCellRenderer(null);VivadoZhRenderers.install(tree,TRANSLATE);
        tree.getCellRenderer().getTreeCellRendererComponent(tree,node,false,false,true,0,false);
        VivadoZhRenderers.restoreAll();check(tree.getCellRenderer()==null,"null tree renderer restored");
    }

    @SuppressWarnings({"unchecked","rawtypes"})
    private static void listsAndCombos() {
        JList<String> list=new JList<>(new String[]{"General","UserSignal"});list.setSelectedIndex(1);
        ListModel<String> model=list.getModel();ListCellRenderer<? super String> original=list.getCellRenderer();
        VivadoZhRenderers.install(list,TRANSLATE);Object wrapper=list.getCellRenderer();
        VivadoZhRenderers.install(list,TRANSLATE);check(wrapper==list.getCellRenderer(),"list idempotent");
        Component rendered=list.getCellRenderer().getListCellRendererComponent(list,"General",0,false,false);
        check("常规".equals(((JLabel)rendered).getText()),"list display translated");
        check(model==list.getModel()&&"General".equals(model.getElementAt(0))&&list.getSelectedIndex()==1,"list data/selection unchanged");
        VivadoZhRenderers.restoreAll();check(list.getCellRenderer()==original,"list renderer restored");

        DelegatingList jx=new DelegatingList();ListCellRenderer<?> stable=jx.getCellRenderer();
        ListCellRenderer<?> inner=jx.getWrappedCellRenderer();
        VivadoZhRenderers.install(jx,TRANSLATE);Object jxWrapper=jx.getWrappedCellRenderer();
        VivadoZhRenderers.install(jx,TRANSLATE);
        check(jx.getCellRenderer()==stable&&jx.getWrappedCellRenderer()==jxWrapper&&jxWrapper!=inner,"JXList wraps inner once");
        rendered=jx.getCellRenderer().getListCellRendererComponent(jx,"General",0,false,false);
        check("常规".equals(((JLabel)rendered).getText()),"JXList rendering has no recursion");
        VivadoZhRenderers.restoreAll();check(jx.getWrappedCellRenderer()==inner,"JXList inner restored");

        JComboBox<String> combo=new JComboBox<>(new String[]{"General","UserSignal"});
        combo.setActionCommand("original-command");combo.setSelectedIndex(1);
        ComboBoxModel<String> comboModel=combo.getModel();ListCellRenderer<? super String> comboOriginal=combo.getRenderer();
        AtomicInteger actions=new AtomicInteger();combo.addActionListener(event -> actions.incrementAndGet());
        VivadoZhRenderers.install(combo,component -> {
            check(VivadoZhRenderers.currentContext().kind==VivadoZhRenderers.Kind.COMBO,"combo context distinguished");
            check(VivadoZhRenderers.currentContext().owner==combo,"combo owner preserved");return TRANSLATE.apply(component);
        });
        rendered=((ListCellRenderer)combo.getRenderer()).getListCellRendererComponent(new JList<>(),"General",-1,false,false);
        check("常规".equals(((JLabel)rendered).getText()),"combo selected display translated");
        check(combo.getModel()==comboModel&&"General".equals(combo.getItemAt(0))&&combo.getSelectedIndex()==1,"combo model/value/selection unchanged");
        check("original-command".equals(combo.getActionCommand())&&actions.get()==0,"combo action command/events unchanged");
        VivadoZhRenderers.restoreAll();check(combo.getRenderer()==comboOriginal,"combo renderer restored");
        combo.setRenderer(null);VivadoZhRenderers.install(combo,TRANSLATE);
        ((ListCellRenderer)combo.getRenderer()).getListCellRendererComponent(new JList<>(),"General",-1,false,false);
        VivadoZhRenderers.restoreAll();check(combo.getRenderer()==null,"null combo renderer restored");
    }

    private static void tablesAndContexts() {
        DefaultTableModel model=new DefaultTableModel(new Object[][]{{"General",Boolean.TRUE}},new Object[]{"Name","Enabled"}) {
            @Override public Class<?> getColumnClass(int column) {return column==1?Boolean.class:String.class;}
        };
        JTable table=new JTable(model);table.setRowSelectionInterval(0,0);table.setColumnSelectionInterval(1,1);
        TableColumn column=table.getColumnModel().getColumn(0);Object identifier=column.getIdentifier();
        AtomicInteger changes=new AtomicInteger();model.addTableModelListener(e -> changes.incrementAndGet());
        TableCellRenderer originalDefault=table.getDefaultRenderer(String.class);
        VivadoZhRenderers.install(table,component -> {
            VivadoZhRenderers.Context context=VivadoZhRenderers.currentContext();
            // A product may safely translate headers while leaving all engineering data exact.
            if(context.kind==VivadoZhRenderers.Kind.TABLE_CELL) return component;
            return TRANSLATE.apply(component);
        });
        TableCellRenderer wrapper=column.getCellRenderer();VivadoZhRenderers.install(table,TRANSLATE);
        check(column.getCellRenderer()==wrapper,"table installation idempotent");
        Component heading=column.getHeaderRenderer().getTableCellRendererComponent(table,"Name",false,false,-1,0);
        check("名称".equals(((JLabel)heading).getText()),"table header display translated");
        Component value=column.getCellRenderer().getTableCellRendererComponent(table,"General",false,false,0,0);
        check("General".equals(((JLabel)value).getText()),"callback can preserve table data display");
        Component bool=table.getCellRenderer(0,1).getTableCellRendererComponent(table,Boolean.TRUE,false,false,0,1);
        check(bool instanceof JCheckBox&&((JCheckBox)bool).isSelected(),"default Boolean renderer retained");
        check(model==table.getModel()&&"General".equals(model.getValueAt(0,0))&&changes.get()==0,"table data/model/events unchanged");
        check("Name".equals(column.getHeaderValue())&&identifier==column.getIdentifier(),"header value and identifier unchanged");
        check(table.getSelectedRow()==0&&table.getSelectedColumn()==1,"table selection unchanged");
        check(table.getDefaultRenderer(String.class)==originalDefault,"table class default unchanged");
        VivadoZhRenderers.restoreAll();
        check(column.getHeaderRenderer()==null&&column.getCellRenderer()==null,"null per-column defaults restored");
        check(VivadoZhRenderers.currentContext()==null,"render context released");
    }

    private static void nestedAndExternalReplacement() {
        JList<String> list=new JList<>(new String[]{"General"});
        list.setCellRenderer((owner,value,index,selected,focus) -> {
            JPanel panel=new JPanel();JPanel inner=new JPanel();inner.add(new JLabel(value));panel.add(inner);return panel;
        });
        VivadoZhRenderers.install(list,TRANSLATE);
        JPanel rendered=(JPanel)list.getCellRenderer().getListCellRendererComponent(list,"General",0,false,false);
        check("常规".equals(((JLabel)((JPanel)rendered.getComponent(0)).getComponent(0)).getText()),"nested renderer labels reached");
        DefaultListCellRenderer replacement=new DefaultListCellRenderer();list.setCellRenderer(replacement);
        VivadoZhRenderers.restoreAll();check(list.getCellRenderer()==replacement,"external list replacement preserved");

        JTable table=new JTable(1,1);VivadoZhRenderers.install(table,TRANSLATE);
        DefaultTableCellRenderer external=new DefaultTableCellRenderer();
        table.getColumnModel().getColumn(0).setCellRenderer(external);
        table.getColumnModel().getColumn(0).setHeaderRenderer(external);
        VivadoZhRenderers.restoreAll();
        check(table.getColumnModel().getColumn(0).getCellRenderer()==external&&table.getColumnModel().getColumn(0).getHeaderRenderer()==external,"external table replacements preserved");

        JList<String> failing=new JList<>(new String[]{"General"});
        VivadoZhRenderers.install(failing,c -> {throw new IllegalArgumentException("expected callback failure");});
        try {failing.getCellRenderer().getListCellRendererComponent(failing,"General",0,false,false);throw new AssertionError("callback should throw");}
        catch(IllegalArgumentException expected) {check(VivadoZhRenderers.currentContext()==null,"context released after callback failure");}
        VivadoZhRenderers.restoreAll();
    }

    private static WeakReference<?> makeCollectible() {
        final class OwnerRetainingRenderer extends DefaultListCellRenderer {
            final JList<String> owner;
            OwnerRetainingRenderer(JList<String> o) {owner=o;}
        }
        JList<String> list=new JList<>(new String[]{"General"});
        list.setCellRenderer(new OwnerRetainingRenderer(list));
        VivadoZhRenderers.install(list,TRANSLATE);return new WeakReference<>(list);
    }

    private static void classLoaderReload() {
        URL classes=VivadoZhRenderersTest.class.getProtectionDomain().getCodeSource().getLocation();
        JList<String> list=new JList<>(new String[]{"General"});
        ListCellRenderer<? super String> original=list.getCellRenderer();
        try(URLClassLoader first=new URLClassLoader(new URL[]{classes},ClassLoader.getPlatformClassLoader());
            URLClassLoader second=new URLClassLoader(new URL[]{classes},ClassLoader.getPlatformClassLoader())) {
            Class<?> firstClass=Class.forName("VivadoZhRenderers",true,first);
            Class<?> secondClass=Class.forName("VivadoZhRenderers",true,second);
            firstClass.getMethod("install",Component.class,UnaryOperator.class).invoke(null,list,UnaryOperator.identity());
            Object firstWrapper=list.getCellRenderer();
            check(firstWrapper.getClass().getClassLoader()==first,"first payload owns wrapper");
            // If an integration forgets restore, the second loader must not nest wrappers.
            secondClass.getMethod("install",Component.class,UnaryOperator.class).invoke(null,list,UnaryOperator.identity());
            check(list.getCellRenderer()==firstWrapper,"cross-loader wrapper nesting prevented");
            firstClass.getMethod("restoreAll").invoke(null);
            check(list.getCellRenderer()==original,"first payload restores original before reload");
            secondClass.getMethod("install",Component.class,UnaryOperator.class).invoke(null,list,UnaryOperator.identity());
            check(list.getCellRenderer().getClass().getClassLoader()==second,"new payload installs cleanly");
            secondClass.getMethod("restoreAll").invoke(null);
            check(list.getCellRenderer()==original,"second payload restores original");
        } catch(Exception error) {throw new AssertionError("classloader reload",error);}
    }

    @SuppressWarnings({"unchecked","rawtypes"})
    private static void installedSwingX() {
        Class<?> treeClass,listClass;
        try {
            treeClass=Class.forName("org.jdesktop.swingx.JXTree");
            listClass=Class.forName("org.jdesktop.swingx.JXList");
        } catch(ClassNotFoundException optionalLibraryAbsent) {return;}
        try {
            JTree tree=(JTree)treeClass.getConstructor().newInstance();
            TreeCellRenderer original=(TreeCellRenderer)treeClass.getMethod("getWrappedCellRenderer").invoke(tree);
            VivadoZhRenderers.install(tree,TRANSLATE);
            Component rendered=tree.getCellRenderer().getTreeCellRendererComponent(tree,new DefaultMutableTreeNode("General"),false,false,true,0,false);
            check(rendered instanceof JLabel&&"常规".equals(((JLabel)rendered).getText()),"installed JXTree actually renders translated text");
            VivadoZhRenderers.restoreAll();
            check(treeClass.getMethod("getWrappedCellRenderer").invoke(tree)==original,"installed JXTree delegate restored");
            JList list=(JList)listClass.getConstructor().newInstance();
            Object listOriginal=listClass.getMethod("getWrappedCellRenderer").invoke(list);
            VivadoZhRenderers.install(list,TRANSLATE);
            rendered=list.getCellRenderer().getListCellRendererComponent(list,"General",0,false,false);
            check(rendered instanceof JLabel&&"常规".equals(((JLabel)rendered).getText()),"installed JXList actually renders translated text");
            VivadoZhRenderers.restoreAll();
            check(listClass.getMethod("getWrappedCellRenderer").invoke(list)==listOriginal,"installed JXList delegate restored");
        } catch(Exception error) {throw new AssertionError("installed SwingX",error);}
    }

    public static void main(String[] args) throws Exception {
        SwingUtilities.invokeAndWait(() -> {trees();listsAndCombos();tablesAndContexts();nestedAndExternalReplacement();classLoaderReload();installedSwingX();});
        final WeakReference<?>[] weak=new WeakReference<?>[1];
        SwingUtilities.invokeAndWait(() -> weak[0]=makeCollectible());
        for(int i=0;i<30&&weak[0].get()!=null;i++) {System.gc();Thread.sleep(30);}
        check(weak[0].get()==null,"registry does not retain owner/delegate cycle");
        SwingUtilities.invokeAndWait(VivadoZhRenderers::restoreAll);
        System.out.println("TEST_PASS VivadoZhRenderersTest checks="+checks);
    }
}
