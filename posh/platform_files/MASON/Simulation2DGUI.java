/**
 * Implemenmtation of Simulation2DGUI.
 * 
 * Please keep the API and its documentation in mason_doc.py in sync!!!
 */

package platform_files.MASON;

// Java imports
import java.util.*;
import java.awt.*;
import javax.swing.JFrame;

// MASON imports
import sim.util.*;
import sim.engine.SimState;
import sim.display.*;
import sim.portrayal.continuous.ContinuousPortrayal2D;
import sim.field.continuous.Continuous2D;

public class Simulation2DGUI extends GUIState {

	public String simName = "";

	public ContinuousPortrayal2D[] portrayalObjs = null;

	public String[] portrayalNames = null;

	public int dwidth = 0;

	public int dheight = 0;

	public Display2D display = null;

	public JFrame displayFrame = null;

	public Simulation2DGUI(Simulation2D sim, String simName, int maxSize,
			String[] portrayalFields) {
		super(sim);
		this.simName = simName;
		assignFields(sim, portrayalFields);
		setDisplaySize(sim, maxSize);
	}

	public Simulation2DGUI(Simulation2D sim, String simName, int maxSize,
			String portrayalField) {
		super(sim);
		this.simName = simName;
		String[] portrayalFields = { portrayalField };
		assignFields(sim, portrayalFields);
		setDisplaySize(sim, maxSize);
	}

	private void assignFields(Simulation2D sim, String[] portrayalFields) {
		// create the portrayals in reverse order of how they are
		// given, as the lsat one is drawn on top of the other portrayals
		portrayalObjs = new ContinuousPortrayal2D[portrayalFields.length];
		portrayalNames = new String[portrayalFields.length];
		for (int i = 0; i < portrayalFields.length; i++) {
			portrayalObjs[i] = new ContinuousPortrayal2D();
			portrayalNames[i] = portrayalFields[portrayalFields.length - i - 1];
			portrayalObjs[i].setField(sim.getField(portrayalNames[i]));
		}
	}

	private void setDisplaySize(Simulation2D sim, int maxSize) {
		if (sim.height > sim.width) {
			dwidth = (int) ((double) (maxSize * sim.width) / (double) sim.height);
			dheight = maxSize;
		} else {
			dwidth = maxSize;
			dheight = (int) ((double) (maxSize * sim.height) / (double) sim.width);
		}
	}

	public static String getName() {
		// from some MASON version < 12 onwards, this method is declared
		// static. Hence, we cannot return simName anymore. :-(
		// called by MASON control
		return "POSH Simulation";
	}

	public void init(Controller controller) {
		super.init(controller);
		setupDisplay(controller);
		// attach the portrayals to the displays
		for (int i = 0; i < portrayalObjs.length; i++)
			display.attach(portrayalObjs[i], portrayalNames[i]);
	}

	public void setupDisplay(Controller controller) {
		createDisplay(controller, simName, Color.black);
	}

	public void createDisplay(Controller controller, String title,
			Paint bgcolour) {
		// use the size we have calculated at construction
		display = new Display2D(dwidth, dheight, this, 1);
		// set the given background colour
		display.setBackdrop(bgcolour);
		// create a frame to display and set its properties
		displayFrame = display.createFrame();
		displayFrame.setTitle(title);
		// register the frame at the controller
		controller.registerFrame(displayFrame);
		// show the frame
		displayFrame.setVisible(true);
	}

	// called by MASON control upon simulation start
	public void start() {
		super.start();
		setupPortrayals();
	}

	public void setupPortrayals() {
		// iterate through all fields and all of its objects,
		// and call each object's getPortrayal() to set its portrayal
		for (int f = 0; f < portrayalNames.length; f++) {
			Continuous2D field = ((Simulation2D) state)
					.getField(portrayalNames[f]);
			Bag objs = field.getAllObjects();
			for (int o = 0; o < objs.numObjs; o++) {
				portrayalObjs[f].setPortrayalForObject(objs.objs[o],
				   ((Entity) objs.objs[o]).getPortrayal());
			}
		}
		// reschedule the displayer
		display.reset();
		// redraw the display
		display.repaint();
	}

	// called upon loading a new checkpoint
	public void load(SimState sim) {
		super.load(sim);
		setupPortrayals();
	}

	public void quit() {
		// called when quitting the SimState
		super.quit();
		if (displayFrame != null) {
			// get rid of the frame
			displayFrame.dispose();
			displayFrame = null;
		}
	}

	public Console run() {
		Console c = new Console(this);
		c.setVisible(true);
		return c;
	}
}
