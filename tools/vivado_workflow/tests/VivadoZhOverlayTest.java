import java.awt.*;
import java.awt.image.BufferedImage;
import java.lang.ref.WeakReference;
import java.util.concurrent.atomic.AtomicInteger;
import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.text.Document;

/** Headless display-adapter tests; never opens or attaches to a real application. */
public final class VivadoZhOverlayTest {
    private static int checks;
    private static void check(boolean condition,String message) {
        checks++;if(!condition) throw new AssertionError(message);
    }
    private static final class Field extends JTextField {
        boolean showing=true;
        boolean focused=false;
        Field(String text) {super(text);setEditable(false);}
        @Override public boolean isShowing() {return showing;}
        @Override public boolean hasFocus() {return focused;}
    }
    private static final class Fixture {
        final JRootPane root=new JRootPane();
        final JPanel content=new JPanel(null);
        final Field field=new Field("Automatically selected checkpoint");
        Fixture() {
            root.setContentPane(content);root.setSize(500,150);root.doLayout();
            content.setBackground(Color.WHITE);field.setBounds(20,30,280,28);
            field.setBackground(Color.WHITE);field.setForeground(Color.BLACK);
            field.setSelectionStart(0);field.setSelectionEnd(0);content.add(field);
        }
        JComponent overlay() {
            for(Component component:root.getLayeredPane().getComponents())
                if(component.getClass().getName().equals("VivadoZhOverlay$Overlay")) return (JComponent)component;
            return null;
        }
    }
    private static BufferedImage paint(JComponent component) {
        BufferedImage image=new BufferedImage(component.getWidth(),component.getHeight(),BufferedImage.TYPE_INT_ARGB);
        Graphics2D graphics=image.createGraphics();component.paint(graphics);graphics.dispose();return image;
    }
    private static int nontransparent(BufferedImage image) {
        int pixels=0;
        for(int y=0;y<image.getHeight();y++) for(int x=0;x<image.getWidth();x++)
            if((image.getRGB(x,y)>>>24)!=0) pixels++;
        return pixels;
    }
    private static void modelAndEvents() {
        Fixture f=new Fixture();Document document=f.field.getDocument();Font font=f.field.getFont();
        f.field.setName("backend-field-id");f.field.setActionCommand("backend-command");
        String[] action={null};f.field.addActionListener(event -> action[0]=event.getActionCommand());
        AtomicInteger changes=new AtomicInteger();
        document.addDocumentListener(new DocumentListener() {
            public void insertUpdate(DocumentEvent e) {changes.incrementAndGet();}
            public void removeUpdate(DocumentEvent e) {changes.incrementAndGet();}
            public void changedUpdate(DocumentEvent e) {changes.incrementAndGet();}
        });
        Component glass=f.root.getGlassPane();int children=f.root.getLayeredPane().getComponentCount();
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");JComponent overlay=f.overlay();
        check(overlay!=null,"overlay attached");
        check(nontransparent(paint(overlay))>0,"Chinese presentation paints inside the field");
        check(f.field.getDocument()==document&&"Automatically selected checkpoint".equals(f.field.getText()),"document and source text preserved");
        check(changes.get()==0&&f.field.getFont()==font&&"backend-field-id".equals(f.field.getName()),"no document event, font or identity mutation");
        check(f.root.getGlassPane()==glass,"original glass pane preserved");
        f.field.postActionEvent();check("backend-command".equals(action[0]),"action command is preserved");
        check(!overlay.isFocusable()&&!overlay.contains(1,1),"overlay cannot take focus or intercept mouse");
        Point inside=SwingUtilities.convertPoint(f.field,10,10,f.root.getLayeredPane());
        check(SwingUtilities.getDeepestComponentAt(f.root.getLayeredPane(),inside.x,inside.y)==f.field,"mouse hit testing passes through to original field");
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");VivadoZhOverlay.refresh();
        check(f.overlay()==overlay&&f.root.getLayeredPane().getComponentCount()==children+1,"offer/refresh is idempotent");
        f.field.select(2,9);VivadoZhOverlay.refresh();
        check(f.field.getSelectionStart()==2&&f.field.getSelectionEnd()==9,"selection preserved");
        check(nontransparent(paint(overlay))==0,"selected source text stays visible and matches clipboard data");
        VivadoZhOverlay.restoreAll();
        check(f.root.getLayeredPane().getComponentCount()==children&&f.overlay()==null,"restoration removes entire adapter");
        check(f.field.getSelectionStart()==2&&f.field.getSelectionEnd()==9&&changes.get()==0,"restoration preserves document and selection");
        VivadoZhOverlay.restoreAll();
    }
    private static void invalidation() {
        Fixture f=new Fixture();VivadoZhOverlay.offer(f.field,"自动选择的检查点");JComponent overlay=f.overlay();
        f.field.setText("different current value");
        check(nontransparent(paint(overlay))==0,"source text change suppresses stale translation immediately");
        f.field.setText("Automatically selected checkpoint");f.field.setEditable(true);
        check(nontransparent(paint(overlay))==0,"editable state suppresses overlay");
        f.field.setEditable(false);f.field.showing=false;
        check(nontransparent(paint(overlay))==0,"hidden field suppresses overlay");
        f.field.showing=true;f.field.setLocation(60,30);
        check(nontransparent(paint(overlay))==0,"unrefreshed movement does not paint at old position");
        VivadoZhOverlay.refresh();check(nontransparent(paint(overlay))>0,"refresh follows field position");
        f.field.setDocument(new javax.swing.text.PlainDocument());f.field.setText("Automatically selected checkpoint");
        check(nontransparent(paint(overlay))==0,"replacement document invalidates the old offer");
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");
        check(nontransparent(paint(overlay))>0,"a fresh exact offer enables replacement document presentation");
        f.content.remove(f.field);VivadoZhOverlay.refresh();check(f.overlay()==null,"detached field overlay removed");
        VivadoZhOverlay.restoreAll();
        f=new Fixture();f.field.setEditable(true);VivadoZhOverlay.offer(f.field,"自动选择的检查点");
        check(f.overlay()==null,"editable field never receives an adapter");
        f.field.setEnabled(false);Document original=f.field.getDocument();String originalText=f.field.getText();
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");JComponent disabledOverlay=f.overlay();
        check(disabledOverlay!=null&&nontransparent(paint(disabledOverlay))>0,"disabled editable field receives a display-only translation");
        check(f.field.getDocument()==original&&f.field.getText().equals(originalText),"disabled field document and original text preserved");
        check(!disabledOverlay.contains(1,1)&&!disabledOverlay.isFocusable(),"disabled field overlay remains mouse-transparent and nonfocusable");
        f.field.setEnabled(true);
        check(nontransparent(paint(disabledOverlay))==0,"re-enabled editable field stops painting before the next scan");
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");
        check(f.overlay()==null,"next offer removes adapter from enabled editable field");
        f.field.setEditable(false);VivadoZhOverlay.offer(f.field,"自动选择的检查点");
        check(f.overlay()!=null&&nontransparent(paint(f.overlay()))>0,"enabled read-only field remains eligible");
        VivadoZhOverlay.restoreAll();
        JPasswordField secret=new JPasswordField("secret");secret.setEditable(false);f.content.add(secret);
        VivadoZhOverlay.offer(secret,"密码");check(f.overlay()==null,"password field never receives an adapter");
        secret.setEnabled(false);secret.setEditable(true);VivadoZhOverlay.offer(secret,"密码");
        check(f.overlay()==null,"disabled password field remains excluded");
        VivadoZhOverlay.restoreAll();
    }

    private static void conservativeDisplayRules() {
        Fixture f=new Fixture();VivadoZhOverlay.offer(f.field,"自动选择的检查点");JComponent overlay=f.overlay();
        f.field.focused=true;check(nontransparent(paint(overlay))==0,"focused field retains original caret/selection feedback");
        f.field.focused=false;f.field.setOpaque(false);
        check(nontransparent(paint(overlay))==0,"transparent target does not borrow ancestor background");f.field.setOpaque(true);
        f.field.setComponentOrientation(ComponentOrientation.RIGHT_TO_LEFT);
        check(nontransparent(paint(overlay))==0,"unverified bidirectional layout is excluded");
        f.field.setComponentOrientation(ComponentOrientation.LEFT_TO_RIGHT);
        VivadoZhOverlay.offer(f.field,"非常非常非常非常非常非常非常非常非常非常非常非常非常非常长的译文");
        check(nontransparent(paint(overlay))==0,"long translation cannot extend over unverified icon space");
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");
        BufferedImage image=paint(overlay);int end;
        try {end=(int)Math.ceil(f.field.getUI().modelToView2D(f.field,f.field.getDocument().getLength(),javax.swing.text.Position.Bias.Backward).getX());}
        catch(javax.swing.text.BadLocationException error) {throw new AssertionError(error);}
        if(end+5<image.getWidth()) check((image.getRGB(end+5,image.getHeight()/2)>>>24)==0,"unoccupied interior/icon region remains untouched");
        f.field.setHorizontalAlignment(SwingConstants.RIGHT);check(nontransparent(paint(overlay))>0,"right-aligned text region supported");
        f.field.setHorizontalAlignment(SwingConstants.CENTER);check(nontransparent(paint(overlay))>0,"center-aligned text region supported");
        f.field.setUI(new javax.swing.plaf.basic.BasicTextFieldUI() {
            @Override public java.awt.geom.Rectangle2D modelToView2D(javax.swing.text.JTextComponent component,int pos,javax.swing.text.Position.Bias bias) {return null;}
        });
        check(nontransparent(paint(overlay))==0,"unknown text geometry is not painted");
        VivadoZhOverlay.restoreAll();
    }
    private static void clipping() {
        Fixture f=new Fixture();JPanel viewport=new JPanel(null);viewport.setBounds(10,10,100,60);
        f.content.remove(f.field);f.content.add(viewport);viewport.add(f.field);f.field.setBounds(0,10,280,28);
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");JComponent overlay=f.overlay();BufferedImage image=paint(overlay);
        check((image.getRGB(10,10)>>>24)!=0,"visible ancestor region paints");
        check((image.getRGB(110,10)>>>24)==0,"ancestor clipping excludes scrolled-out text");
        Insets insets=f.field.getInsets();
        if(insets.left>0) check((image.getRGB(0,10)>>>24)==0,"field border remains uncovered");
        JPanel covering=new JPanel();covering.setBounds(0,10,50,28);viewport.add(covering,0);
        image=paint(overlay);
        check((image.getRGB(10,10)>>>24)==0&&(image.getRGB(70,10)>>>24)!=0,"foreground sibling control is not painted over");
        JPanel popup=new JPanel();popup.setBounds(10,20,100,28);
        f.root.getLayeredPane().add(popup,JLayeredPane.POPUP_LAYER);
        image=paint(overlay);check(nontransparent(image)==0,"popup layer remains above and excludes overlay");
        VivadoZhOverlay.restoreAll();
    }
    private static void actualLookAndFeelBackground() {
        Fixture f=new Fixture();Color paintedGray=new Color(218,221,225);
        f.field.setEditable(true);f.field.setEnabled(false);Document document=f.field.getDocument();
        String original=f.field.getText();
        f.field.setUI(new javax.swing.plaf.basic.BasicTextFieldUI() {
            @Override protected void paintBackground(Graphics graphics) {
                graphics.setColor(paintedGray);graphics.fillRect(0,0,getComponent().getWidth(),getComponent().getHeight());
            }
        });
        check(Color.WHITE.equals(f.field.getBackground()),"test LAF paint differs from background property");
        VivadoZhOverlay.offer(f.field,"自动选择的检查点");BufferedImage image=paint(f.overlay());
        int matching=0,white=0;
        for(int y=0;y<image.getHeight();y++) for(int x=0;x<image.getWidth();x++) {
            if(image.getRGB(x,y)==paintedGray.getRGB()) matching++;
            if(image.getRGB(x,y)==Color.WHITE.getRGB()) white++;
        }
        check(matching>100&&white==0,"overlay uses actual LAF gray without a property-color white block");
        check(f.field.getDocument()==document&&original.equals(f.field.getText())&&Color.WHITE.equals(f.field.getBackground()),"background sampling leaves document/text/property unchanged");
        f.field.setUI(new javax.swing.plaf.basic.BasicTextFieldUI() {
            @Override protected void paintBackground(Graphics graphics) {
                for(int x=0;x<getComponent().getWidth();x++) {
                    graphics.setColor(new Color(150+x%100,160,170));graphics.drawLine(x,0,x,getComponent().getHeight());
                }
            }
        });
        check(nontransparent(paint(f.overlay()))==0,"nonuniform painted background is rejected");
        VivadoZhOverlay.restoreAll();
    }
    private static WeakReference<Field> collectible(JRootPane[] retained) {
        Fixture f=new Fixture();VivadoZhOverlay.offer(f.field,"自动选择的检查点");
        f.content.remove(f.field);retained[0]=f.root;return new WeakReference<>(f.field);
    }
    public static void main(String[] args) throws Exception {
        try {VivadoZhOverlay.refresh();throw new AssertionError("EDT restriction missing");}
        catch(IllegalStateException expected) {check(true,"off-EDT access rejected");}
        SwingUtilities.invokeAndWait(() -> {modelAndEvents();invalidation();clipping();conservativeDisplayRules();actualLookAndFeelBackground();});
        JRootPane[] retained=new JRootPane[1];WeakReference<?>[] weak=new WeakReference<?>[1];
        SwingUtilities.invokeAndWait(() -> weak[0]=collectible(retained));
        for(int i=0;i<20&&weak[0].get()!=null;i++) {System.gc();Thread.sleep(30);}
        check(weak[0].get()==null,"overlay does not retain detached field/document");
        SwingUtilities.invokeAndWait(() -> {
            VivadoZhOverlay.refresh();check(retained[0].getLayeredPane().getComponentCount()==1,"refresh removes orphan overlay after collection");
            VivadoZhOverlay.restoreAll();
        });
        System.out.println("VivadoZhOverlayTest PASS: "+checks+" checks");
    }
}
