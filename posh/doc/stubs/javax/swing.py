"""JavaX Swing Stub.

based on http://java.sun.com/j2se/1.3/docs/api/javax/swing/package-tree.html.
"""

from java.lang import Object, Exception
from java.util import Hashtable
from java.awt import Component, Container, Dialog, Frame, Graphics
from java.applet import Applet

class AbstractAction(Object):
    pass

class AbstractCellEditor(Object):
    pass

class DefaultCellEditor(AbstractCellEditor):
    pass

class AbstractListModel(Object):
    pass

class DefaultComboBoxModel(AbstractListModel):
    pass

class DefaultListModel(AbstractListModel):
    pass

class AccessibleContext(Object):
    pass

class ActionMap(Object):
    pass

class BorderFactory(Object):
    pass

class BoxLayout(Object):
    pass

class ButtonGroup(Object):
    pass

class Box(Container):
    pass

class CellRenderPane(Container):
    pass

class JComponent(Container):
    pass

class AbstractButton(JComponent):
    pass

class JButton(AbstractButton):
    pass

class JMenuItem(AbstractButton):
    pass

class JCheckBoxMenuItem(JMenuItem):
    pass

class JMenu(JMenuItem):
    pass

class JRadioButtonMenuItem(JMenuItem):
    pass

class JToggleButton(AbstractButton):
    pass

class JCheckBox(JToggleButton):
    pass

class JRadioButton(JToggleButton):
    pass

class JColorChooser(JComponent):
    pass

class JComboBox(JComponent):
    pass

class JFileChooser(JComponent):
    pass

class JInternalFrame(JComponent):
    pass

class JLabel(JComponent):
    pass

class DefaultListCellRenderer(JLabel):
    pass

class JLayeredPane(JComponent):
    pass

class JDesktopPane(JLayeredPane):
    pass

class JList(JComponent):
    pass

class JMenuBar(JComponent):
    pass

class JOptionPane(JComponent):
    pass

class JPanel(JComponent):
    pass

class JPopupMenu(JComponent):
    pass

class JProgressBar(JComponent):
    pass

class JRootPane(JComponent):
    pass

class JScrollBar(JComponent):
    pass

class JScrollPane(JComponent):
    pass

class JSeparator(JComponent):
    pass

class JSlider(JComponent):
    pass

class JSplitPane(JComponent):
    pass

class JTabbedPane(JComponent):
    pass

class JTable(JComponent):
    pass

# not correct
class JEditorPane(JComponent):
    pass

class JTextPane(JEditorPane):
    pass

# not correct
class JTextArea(JComponent):
    pass

# not correct
class JTextField(JComponent):
    pass

class JPasswordField(JTextField):
    pass

class JToolBar(JComponent):
    pass

class JToolTip(JComponent):
    pass

class JTree(JComponent):
    pass

class JViewport(JComponent):
    pass

class JApplet(Applet):
    pass

class JDialog(Dialog):
    pass

class JFrame(Frame):
    pass

class JWindow(Frame):
    pass

class DefaultBoundedRangeModel(Object):
    pass

class DefaultButtonModel(Object):
    pass

class DefaultDesktopManager(Object):
    pass

class DefaultListSelectionModel(Object):
    pass

class DefaultSingleSelectionModel(Object):
    pass

class UIDefaults(Hashtable):
    pass

class FocusManager(Object):
    pass

class DefaultFocusManager(FocusManager):
    pass

class DebugGraphics(Graphics):
    pass

class ImageIcon(Object):
    pass

class InputMap(Object):
    pass

class ComponentInputMap(InputMap):
    pass

class InputVerifier(Object):
    pass

class KeyStroke(Object):
    pass

class LookAndFeel(Object):
    pass

class MenuSelectionManager(Object):
    pass

class OverlayLayout(Object):
    pass

class ProgressMonitor(Object):
    pass

class RepaintManager(Object):
    pass

class ScrollPaneLayout(Object):
    pass

class SizeRequirements(Object):
    pass

class SizeSequence(Object):
    pass

class SwingUtilities(Object):
    pass

class UnsupportedLookAndFeelException(Exception):
    pass

class Timer(Object):
    pass

class UIManager(Object):
    pass

class ViewportLayout(Object):
    pass

