import java.awt.*;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;
import java.util.Objects;
import java.util.WeakHashMap;
import javax.swing.*;
import javax.swing.plaf.TextUI;
import javax.swing.text.BadLocationException;
import javax.swing.text.Document;
import javax.swing.text.Position;

/**
 * Display-only translation of approved, read-only or disabled single-line fields. The caller
 * must supply an exact reviewed dictionary translation; this class has no lookup
 * or machine-translation path. It never changes the text, document, caret,
 * selection, component font, layout, glass pane, or event handling of a field.
 */
public final class VivadoZhOverlay {
    private static final Map<JTextField,WeakReference<Overlay>> fields=new WeakHashMap<>();
    // A field can be collected before its old layer is removed. Retain a weak
    // index of every overlay so refresh/restore can still remove those orphans.
    private static final ArrayList<WeakReference<Overlay>> layers=new ArrayList<>();

    private VivadoZhOverlay() {}

    /** All three public operations must run on the Swing EDT. */
    public static void offer(JTextField field,String translated) {
        requireEdt();
        if(field==null) return;
        Overlay previous=dereference(fields.get(field));
        if(field instanceof JPasswordField || (field.isEditable()&&field.isEnabled()) || translated==null
                || translated.isEmpty() || translated.indexOf('\n')>=0 || Objects.equals(field.getText(),translated)) {
            if(previous!=null) previous.remove();
            fields.remove(field);return;
        }
        JRootPane root=SwingUtilities.getRootPane(field);
        if(root==null || root.getLayeredPane()==null) {
            if(previous!=null) previous.remove();
            fields.remove(field);return;
        }
        JLayeredPane host=root.getLayeredPane();
        if(previous==null || previous.getParent()!=host) {
            if(previous!=null) previous.remove();
            previous=new Overlay(field);
            // Above ordinary content, below popups, drag layers and glass panes.
            // Derive from the actual root content layer instead of assuming 0.
            int contentLayer=host.getLayer(root.getContentPane());
            if(contentLayer+1>=JLayeredPane.PALETTE_LAYER) return;
            host.add(previous,Integer.valueOf(contentLayer+1),0);
            fields.put(field,new WeakReference<>(previous));
            layers.add(new WeakReference<>(previous));
        }
        previous.original=field.getText();
        previous.document=new WeakReference<>(field.getDocument());
        previous.translated=translated;
        previous.refresh();
    }

    public static void refresh() {
        requireEdt();
        Iterator<WeakReference<Overlay>> iterator=layers.iterator();
        while(iterator.hasNext()) {
            Overlay layer=iterator.next().get();
            if(layer==null) {iterator.remove();continue;}
            JTextField field=layer.field.get();
            if(field==null || layer.getParent()==null
                    || SwingUtilities.getRootPane(field)==null
                    || SwingUtilities.getRootPane(field).getLayeredPane()!=layer.getParent()) {
                layer.remove();iterator.remove();
                if(field!=null && dereference(fields.get(field))==layer) fields.remove(field);
            } else layer.refresh();
        }
    }

    public static void restoreAll() {
        requireEdt();
        for(WeakReference<Overlay> reference:layers) {
            Overlay layer=reference.get();if(layer!=null) layer.remove();
        }
        fields.clear();layers.clear();
    }

    private static Overlay dereference(WeakReference<Overlay> ref) {return ref==null?null:ref.get();}
    private static void requireEdt() {
        if(!SwingUtilities.isEventDispatchThread())
            throw new IllegalStateException("Read-only field translation requires the Swing EDT");
    }

    private static final class Overlay extends JComponent {
        final WeakReference<JTextField> field;
        WeakReference<Document> document=new WeakReference<>(null);
        String original,translated;

        Overlay(JTextField target) {
            field=new WeakReference<>(target);
            setOpaque(false);setFocusable(false);setRequestFocusEnabled(false);
            setDoubleBuffered(false);
        }

        // Swing's deepest-component hit testing therefore reaches the original
        // field, including selection, context menus, tooltips and mouse wheel.
        @Override public boolean contains(int x,int y) {return false;}

        void refresh() {
            JTextField target=field.get();Container host=getParent();
            if(target==null||host==null) return;
            Rectangle old=getBounds();
            Rectangle next=SwingUtilities.convertRectangle(target,new Rectangle(0,0,target.getWidth(),target.getHeight()),host);
            if(!next.equals(old)) {
                setBounds(next);host.repaint(old.x,old.y,old.width,old.height);
            }
            // A stale translation is never painted; the underlying field shows
            // its original/current text until the caller offers a new mapping.
            repaint();
        }

        void remove() {
            Container host=getParent();
            if(host!=null) {
                Rectangle old=getBounds();host.remove(this);
                host.repaint(old.x,old.y,old.width,old.height);
            }
        }

        private boolean eligible(JTextField target) {
            if(target==null || target instanceof JPasswordField || (target.isEditable()&&target.isEnabled())
                    || !target.isShowing() || target.getDocument()!=document.get()
                    || !target.isOpaque() || target.hasFocus()
                    || target.getSelectionStart()!=target.getSelectionEnd()
                    || !target.getComponentOrientation().isLeftToRight()
                    || !Objects.equals(original,target.getText())) return false;
            JRootPane root=SwingUtilities.getRootPane(target);
            return root!=null && root.getLayeredPane()==getParent();
        }

        /** Clip to every ancestor and exclude foreground siblings conservatively. */
        private Area visibleArea(JTextField target,Rectangle inner) {
            Area area=new Area(SwingUtilities.convertRectangle(target,inner,this));
            Component child=target;
            for(Container parent=child.getParent();parent!=null;child=parent,parent=parent.getParent()) {
                if(!parent.isVisible()) return new Area();
                area.intersect(new Area(SwingUtilities.convertRectangle(parent,
                        new Rectangle(0,0,parent.getWidth(),parent.getHeight()),this)));
                int index=parent.getComponentZOrder(child);
                for(int i=0;i<index;i++) {
                    Component sibling=parent.getComponent(i);
                    if(sibling==this || sibling instanceof Overlay || !sibling.isVisible()) continue;
                    // Even a transparent foreground sibling is excluded: hiding
                    // some translation is preferable to painting over controls.
                    area.subtract(new Area(SwingUtilities.convertRectangle(parent,sibling.getBounds(),this)));
                }
                if(parent==getParent()) return area;
            }
            return new Area();
        }

        /**
         * Sample the actual LAF paint, not getBackground(): Synthetica paints
         * disabled fields gray even when that property remains white. The layer
         * is a sibling of the field, so painting the field cannot recurse here.
         */
        private Color paintedBackground(JTextField target,Rectangle inner,Rectangle erase,int textStart,int textEnd) {
            int width=target.getWidth(),height=target.getHeight();
            if(width<=0||height<=0||width>4096||height>128||(long)width*height>262144) return null;
            int sampleTop=Math.max(inner.y+1,erase.y+1);
            int sampleBottom=Math.min(inner.y+inner.height-1,erase.y+erase.height-1);
            if(sampleBottom-sampleTop<4) return null;
            int[][] strips={
                {inner.x+1,Math.min(inner.x+inner.width-1,textStart-2)},
                {Math.max(inner.x+1,textEnd+2),inner.x+inner.width-1}
            };
            boolean hasSpace=false;
            for(int[] strip:strips) if(strip[1]-strip[0]>=3) hasSpace=true;
            if(!hasSpace) return null;
            BufferedImage image=new BufferedImage(width,height,BufferedImage.TYPE_INT_ARGB);
            Graphics2D probe=image.createGraphics();
            try {target.paint(probe);} finally {probe.dispose();}
            try {
                Integer candidate=null;int samples=0;
                // All samples are outside the original glyph interval and inside
                // the border. A gradient, icon or inconsistent pattern fails closed.
                for(int[] strip:strips) if(strip[1]-strip[0]>=3)
                    for(int y=sampleTop;y<sampleBottom;y++) for(int x=strip[0];x<strip[1];x++) {
                        int pixel=image.getRGB(x,y);
                        if((pixel>>>24)!=255 || (candidate!=null&&pixel!=candidate)) return null;
                        candidate=pixel;samples++;
                    }
                if(candidate==null||samples<12) return null;
                int matching=0,total=0;
                // Confirm that the sampled color is also the dominant background
                // underneath the original text, rather than a differently colored
                // suffix area. Foreground glyph and antialiasing pixels are allowed.
                for(int y=erase.y;y<erase.y+erase.height;y++) for(int x=erase.x;x<erase.x+erase.width;x++) {
                    if(image.getRGB(x,y)==candidate) matching++;total++;
                }
                if(total==0||matching*100L<total*60L) return null;
                return new Color(candidate,true);
            } finally {image.flush();}
        }

        @Override protected void paintComponent(Graphics graphics) {
            JTextField target=field.get();
            if(!eligible(target)) return;
            Rectangle current=SwingUtilities.convertRectangle(target,
                    new Rectangle(0,0,target.getWidth(),target.getHeight()),getParent());
            // Geometry can change between timer scans; do not paint at stale coordinates.
            if(!current.equals(getBounds())) return;
            Insets insets=target.getInsets();
            Rectangle inner=new Rectangle(insets.left,insets.top,
                    target.getWidth()-insets.left-insets.right,target.getHeight()-insets.top-insets.bottom);
            if(inner.width<=0||inner.height<=0) return;
            Area clip=visibleArea(target,inner);if(clip.isEmpty()) return;

            TextUI ui=target.getUI();if(ui==null) return;
            Font font=target.getFont();if(font==null) return;
            if(font.canDisplayUpTo(translated)>=0)
                font=new Font("Noto Sans CJK SC",font.getStyle(),font.getSize()).deriveFont(font.getSize2D());
            if(font.canDisplayUpTo(translated)>=0) return;
            Rectangle2D start,end;
            try {
                start=ui.modelToView2D(target,0,Position.Bias.Forward);
                end=ui.modelToView2D(target,target.getDocument().getLength(),Position.Bias.Backward);
            } catch(BadLocationException invalidLayout) {return;}
            if(start==null||end==null||Math.abs(start.getY()-end.getY())>1||end.getX()<=start.getX()) return;
            FontMetrics metrics=target.getFontMetrics(font);int textWidth=metrics.stringWidth(translated);
            int textStart=(int)Math.floor(start.getX()),textEnd=(int)Math.ceil(end.getX());
            int visibleStart=Math.max(textStart,inner.x),visibleEnd=Math.min(textEnd,inner.x+inner.width);
            // Never extend into unverified interior space that may hold an icon.
            if(visibleStart>=visibleEnd||textWidth>visibleEnd-visibleStart) return;
            int x=visibleStart,align=target.getHorizontalAlignment();
            if(align==SwingConstants.CENTER) x+=(visibleEnd-visibleStart-textWidth)/2;
            else if(align==SwingConstants.RIGHT||align==SwingConstants.TRAILING) x=visibleEnd-textWidth;
            int y=target.getBaseline(target.getWidth(),target.getHeight());
            if(y<0) y=(int)Math.round(start.getY()+(start.getHeight()-metrics.getHeight())/2)+metrics.getAscent();
            int top=(int)Math.floor(start.getY()),bottom=(int)Math.ceil(start.getY()+start.getHeight());
            Rectangle erase=new Rectangle(textStart,top,Math.max(1,textEnd-textStart+1),Math.max(1,bottom-top)).intersection(inner);
            Color background=paintedBackground(target,inner,erase,textStart,textEnd);
            if(background==null) return;
            Graphics2D g=(Graphics2D)graphics.create();
            try {
                g.clip(clip);g.setColor(background);g.fill(erase);g.setFont(font);
                Object hints=Toolkit.getDefaultToolkit().getDesktopProperty("awt.font.desktophints");
                if(hints instanceof Map<?,?>) g.addRenderingHints((Map<?,?>)hints);
                else g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
                Color foreground=target.isEnabled()?target.getForeground():target.getDisabledTextColor();
                if(foreground==null) foreground=UIManager.getColor("TextField.foreground");
                g.setColor(foreground==null?Color.BLACK:foreground);
                g.drawString(translated,x,y);
            } finally {g.dispose();}
        }
    }
}
