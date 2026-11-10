import java.awt.*;
import java.awt.image.BufferedImage;
import java.lang.reflect.Array;
import javax.swing.*;
import javax.swing.border.Border;

/** Run with the installed Vivado jide-common jar on the classpath; no GUI required. */
public final class VivadoZhStyledLabelTest {
    private static int checks;
    private static void check(boolean ok,String label) {checks++;if(!ok)throw new AssertionError(label);}

    private static void test() {
        try {
            Class<?> styledClass=Class.forName("com.jidesoft.swing.StyledLabel");
            Class<?> styleClass=Class.forName("com.jidesoft.swing.StyleRange");
            String originalText="Design Sources (19 items)";
            JLabel original=(JLabel)styledClass.getConstructor(String.class).newInstance(originalText);
            Object range=styleClass.getConstructor(int.class,int.class,int.class,Color.class)
                .newInstance(19,5,Font.BOLD,Color.RED);
            Object ranges=Array.newInstance(styleClass,1);Array.set(ranges,0,range);
            styledClass.getMethod("setStyleRanges",ranges.getClass()).invoke(original,ranges);
            Icon icon=new ImageIcon(new BufferedImage(8,8,BufferedImage.TYPE_INT_ARGB));
            Icon disabled=new ImageIcon(new BufferedImage(7,7,BufferedImage.TYPE_INT_ARGB));
            Border border=BorderFactory.createEmptyBorder(1,2,3,4);
            original.setIcon(icon);original.setDisabledIcon(disabled);
            original.setFont(new Font("Liberation Sans",Font.ITALIC,14));
            original.setForeground(Color.BLUE);original.setBackground(Color.YELLOW);original.setBorder(border);
            original.setHorizontalAlignment(SwingConstants.TRAILING);original.setVerticalAlignment(SwingConstants.BOTTOM);
            original.setHorizontalTextPosition(SwingConstants.LEADING);original.setVerticalTextPosition(SwingConstants.TOP);
            original.setIconTextGap(9);original.setOpaque(true);original.setEnabled(false);
            original.setToolTipText("Original tooltip");original.setComponentOrientation(ComponentOrientation.RIGHT_TO_LEFT);
            Font originalFont=original.getFont();
            Object before=styledClass.getMethod("getStyleRanges").invoke(original);
            check(VivadoZhStyledLabel.isStyledLabel(original),"detect actual installed StyledLabel");
            check(!VivadoZhStyledLabel.isStyledLabel(new JLabel())&&!VivadoZhStyledLabel.isStyledLabel(null),"ordinary/null components excluded");

            JLabel copy=(JLabel)VivadoZhStyledLabel.translatedCopy(original,"设计源文件");
            check(copy.getClass()==JLabel.class&&copy!=original,"copy is an independent plain JLabel");
            check("设计源文件".equals(copy.getText()),"Chinese display text installed only on copy");
            check(originalText.equals(original.getText()),"original text unchanged");
            Object after=styledClass.getMethod("getStyleRanges").invoke(original);
            check(Array.getLength(after)==1&&Array.get(before,0)==Array.get(after,0),"original style objects unchanged");
            check((int)styleClass.getMethod("getStart").invoke(range)==19&&(int)styleClass.getMethod("getLength").invoke(range)==5,"original style offsets unchanged");
            check(original.getFont()==originalFont,"original font unchanged");
            check(copy.getIcon()==icon&&copy.getDisabledIcon()==disabled,"both icons copied");
            check(copy.getBorder()==border&&copy.getForeground()==Color.BLUE&&copy.getBackground()==Color.YELLOW,"border and colors copied");
            check(copy.getHorizontalAlignment()==SwingConstants.TRAILING&&copy.getVerticalAlignment()==SwingConstants.BOTTOM,"alignment copied");
            check(copy.getHorizontalTextPosition()==SwingConstants.LEADING&&copy.getVerticalTextPosition()==SwingConstants.TOP,"text positions copied");
            check(copy.getIconTextGap()==9&&copy.isOpaque()&&!copy.isEnabled(),"gap and states copied");
            check("Original tooltip".equals(copy.getToolTipText())&&!copy.getComponentOrientation().isLeftToRight(),"tooltip and orientation copied");
            check(copy.getFont().getStyle()==originalFont.getStyle()&&copy.getFont().getSize2D()==originalFont.getSize2D(),"font style and size preserved");
            check(copy.getFont().canDisplayUpTo(copy.getText())<0,"font displays Chinese text");
            Dimension size=copy.getPreferredSize();
            check(size.width>0&&size.height>0,"Chinese copy computes preferred size despite original style offset 19");
            copy.setSize(size);BufferedImage canvas=new BufferedImage(size.width,size.height,BufferedImage.TYPE_INT_ARGB);
            Graphics2D graphics=canvas.createGraphics();try {copy.paint(graphics);}finally{graphics.dispose();}
            check(originalText.equals(original.getText()),"painting copy preserves original text");
            // Reusing the original renderer with another translation cannot corrupt styles.
            JLabel shorter=(JLabel)VivadoZhStyledLabel.translatedCopy(original,"源");
            check(shorter.getPreferredSize().width>0&&originalText.equals(original.getText()),"one-character copy handles long original style ranges");
            JLabel html=(JLabel)VivadoZhStyledLabel.translatedCopy(original,"<html><b>设计源文件</b></html>");
            check(html.getPreferredSize().width>0,"HTML Chinese copy also computes layout");

            // Static headings use one coupled snapshot, never separate text/range changes.
            original.setName("LanguageOptionsHeading");
            original.addPropertyChangeListener("text",event -> {
                // Vivado subclasses can derive automation names and request layout
                // synchronously from setText; both cases must remain safe.
                original.setName("DerivedFromTranslatedText");original.getPreferredSize();
            });
            VivadoZhStyledLabel.Snapshot english=VivadoZhStyledLabel.read(original);
            VivadoZhStyledLabel.Snapshot englishAgain=VivadoZhStyledLabel.read(original);
            check(english.equals(englishAgain)&&english.hashCode()==englishAgain.hashCode(),"equal snapshots compare text and original range references");
            Object[] exposed=english.styleRanges();exposed[0]=null;
            check(english.styleRanges()[0]==range,"snapshot array cannot be mutated through accessor");
            VivadoZhStyledLabel.Snapshot chinese=VivadoZhStyledLabel.translated(english,"语言选项");
            check(chinese.styleCount()==0&&"语言选项".equals(chinese.text)&&!chinese.equals(english),"translated snapshot drops English offsets");
            VivadoZhStyledLabel.apply(original,chinese);
            check(VivadoZhStyledLabel.read(original).equals(chinese),"English to Chinese applies coupled state");
            check(original.getPreferredSize().width>0,"actual StyledLabel Chinese heading computes layout");
            check("LanguageOptionsHeading".equals(original.getName()),"static component name preserved");
            VivadoZhStyledLabel.apply(original,english);
            Object restoredRanges=styledClass.getMethod("getStyleRanges").invoke(original);
            check(VivadoZhStyledLabel.read(original).equals(english)&&Array.get(restoredRanges,0)==range,"Chinese to English restores original range identity");
            check(original.getPreferredSize().width>0,"restored English range remains valid");

            // Model the earlier failure: the current text became shorter but kept the
            // old offset 19. Applying a valid snapshot must clear that offset first.
            try {original.setText("源");} catch(StringIndexOutOfBoundsException expectedOldFailure) {}
            original.setName("LanguageOptionsHeading");
            VivadoZhStyledLabel.apply(original,chinese);
            check("语言选项".equals(original.getText())&&original.getPreferredSize().width>0,"snapshot apply repairs stale offset state");
            VivadoZhStyledLabel.apply(original,english);
            check(original.getPreferredSize().width>0&&VivadoZhStyledLabel.read(original).equals(english),"repaired heading restores English again");
            for(int i=0;i<3;i++) {
                VivadoZhStyledLabel.apply(original,chinese);original.getPreferredSize();
                VivadoZhStyledLabel.apply(original,english);original.getPreferredSize();
            }
            check(english.styleRanges()[0]==range&&(int)styleClass.getMethod("getStart").invoke(range)==19,"repeated switching never mutates original StyleRange");
        } catch(Exception error) {throw new AssertionError("Installed JIDE styled-label regression",error);}
    }

    public static void main(String[] args) throws Exception {
        SwingUtilities.invokeAndWait(VivadoZhStyledLabelTest::test);
        System.out.println("TEST_PASS VivadoZhStyledLabelTest checks="+checks);
    }
}
