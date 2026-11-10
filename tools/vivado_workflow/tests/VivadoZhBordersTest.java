import java.awt.Font;
import java.lang.ref.WeakReference;
import java.lang.reflect.Method;
import java.util.Map;
import java.util.function.UnaryOperator;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.border.Border;
import javax.swing.border.CompoundBorder;
import javax.swing.border.EmptyBorder;
import javax.swing.border.TitledBorder;

/** Uses the real installed planAhead.jar classes without launching Vivado. */
public final class VivadoZhBordersTest {
    private static int checks;
    private static final Map<String,String> WORDS=Map.of(
        "Language Options","语言选项","Scaling","缩放","Spacing","间距",
        "Rendering","渲染","Messages","消息","Hyperlinks","超链接");
    private static final UnaryOperator<String> TRANSLATE=s -> s==null?null:WORDS.getOrDefault(s,s);

    public static void main(String[] args) throws Exception {
        SwingUtilities.invokeAndWait(() -> {
            try {realVivadoBorders();standardNested();outsideChanges();}
            catch(Exception e) {throw new RuntimeException(e);}
        });
        final WeakReference<?>[] references=new WeakReference<?>[2];
        SwingUtilities.invokeAndWait(() -> {
            JPanel owner=new JPanel();TitledBorder border=new TitledBorder("Scaling");owner.setBorder(border);
            VivadoZhBorders.install(owner,TRANSLATE);
            references[0]=new WeakReference<>(owner);references[1]=new WeakReference<>(border);
        });
        for(int i=0;i<30&&(references[0].get()!=null||references[1].get()!=null);i++) {System.gc();Thread.sleep(20);}
        check(references[0].get()==null,"closed owner is collectible");
        check(references[1].get()==null,"border is collectible");
        SwingUtilities.invokeAndWait(VivadoZhBorders::restoreAll);
        boolean guarded=false;try {VivadoZhBorders.install(new JPanel(),TRANSLATE);}catch(IllegalStateException expected){guarded=true;}
        check(guarded,"EDT guard");
        System.out.println("TEST_PASS VivadoZhBordersTest checks="+checks);
    }

    private static void realVivadoBorders() throws Exception {
        Class<?> type=Class.forName("ui.utils.swing.a.y");
        Method getTitle=type.getMethod("getTitle"),getBorder=type.getMethod("getBorder"),getFont=type.getMethod("jPh");
        for(Map.Entry<String,String> word:WORDS.entrySet()) {
            Border border=(Border)type.getConstructor(String.class).newInstance(word.getKey());
            Object inside=getBorder.invoke(border),font=getFont.invoke(border);
            JPanel owner=new JPanel();owner.setName("user_panel");owner.setBorder(new CompoundBorder(new EmptyBorder(1,2,3,4),border));
            Border original=owner.getBorder();
            VivadoZhBorders.install(owner,TRANSLATE);
            check(getTitle.invoke(border).equals(word.getValue()),"actual Vivado y title translated: "+word.getKey());
            check(owner.getBorder()==original,"border wrapper identities preserved");
            check(getBorder.invoke(border)==inside,"custom inner border preserved");
            check(owner.getName().equals("user_panel"),"component identifier unchanged");
            VivadoZhBorders.install(owner,TRANSLATE);
            VivadoZhBorders.restoreAll();
            check(getTitle.invoke(border).equals(word.getKey()),"actual title restored");
            check(java.util.Objects.equals(getFont.invoke(border),font),"actual nullable font restored");
        }
    }

    private static void standardNested() {
        RawTitledBorder inner=new RawTitledBorder("Messages");
        TitledBorder outer=new TitledBorder(new CompoundBorder(new EmptyBorder(1,1,1,1),inner),"Scaling");
        Font original=new Font("Dialog",Font.BOLD,13);outer.setTitleFont(original);
        JPanel owner=new JPanel();owner.setBorder(outer);
        VivadoZhBorders.install(owner,TRANSLATE);
        check(outer.getTitle().equals("缩放"),"standard title translated");
        check(inner.getTitle().equals("消息"),"nested standard title translated");
        check(inner.rawFont()==null,"implicit standard font remains implicit during translation");
        VivadoZhBorders.restoreAll();
        check(outer.getTitle().equals("Scaling")&&inner.getTitle().equals("Messages"),"nested titles restored");
        check(outer.getTitleFont().equals(original),"explicit font restored");
        check(inner.rawFont()==null,"implicit standard font stays null after restoration");
    }

    private static void outsideChanges() {
        JPanel owner=new JPanel();TitledBorder border=new TitledBorder("Scaling");owner.setBorder(border);
        VivadoZhBorders.install(owner,TRANSLATE);
        border.setTitle("external title");
        VivadoZhBorders.restoreAll();
        check(border.getTitle().equals("external title"),"external title change survives rollback");
        border.setTitle("Scaling");VivadoZhBorders.install(owner,TRANSLATE);
        border.setTitle("Spacing");VivadoZhBorders.install(owner,TRANSLATE);
        check(border.getTitle().equals("间距"),"new dynamic title translated");
        VivadoZhBorders.restoreAll();
        check(border.getTitle().equals("Spacing"),"rollback restores newest external title");
    }

    private static void check(boolean condition,String description) {
        if(!condition) throw new AssertionError(description);checks++;
    }

    private static final class RawTitledBorder extends TitledBorder {
        RawTitledBorder(String title) {super(title);}
        Font rawFont() {return titleFont;}
    }
}
