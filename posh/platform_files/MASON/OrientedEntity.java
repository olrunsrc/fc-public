/**
 * Implementation of OrientedEntity.
 * 
 * Please keep the API and its documentation in mason_doc.py in sync!!!
 */

package platform_files.MASON;

// Java imports
import java.lang.Math;
import java.awt.Color;

// MASON imports
import sim.util.*;
import sim.portrayal.*;
import sim.portrayal.simple.*;


public class OrientedEntity extends Entity implements Oriented2D {
	
	public double ori = 0.0;
	
	public OrientedEntity(Simulation2D sim, String[] fieldNames,
			Double2D location, double orientation) {
		super(sim, fieldNames, location);
		ori = orientation;
	}
	
	public OrientedEntity(Simulation2D sim, String[] fieldNames,
			Double2D location) {
		super(sim, fieldNames, location);
		randomiseOrientation();
	}
	
	public OrientedEntity(Simulation2D sim, String[] fieldNames) {
		super(sim, fieldNames);
		randomiseOrientation();
	}

	public OrientedEntity(Simulation2D sim) {
		super(sim);
		randomiseOrientation();
	}
	
	public void setLoc(Double2D loc) {
		Double2D prev = null;
		// at initialisation the location is not assigned
		if (this.loc == null) {
			prev = new Double2D(0.0, 0.0);
		} else {
			prev = new Double2D(this.loc.x, this.loc.y);
		}
		super.setLoc(loc);
		// correct the orientation, based on the current movement
		Double2D v = fields[0].tv(loc, prev);
		ori = Math.atan2(v.y, v.x);
	}
	
	// this method is required to allow 'loc' attribute access from
	// jython (as setLoc() is defined, it thinks that it needs
	// a getLoc() in the same level, otherwise it treats 'loc' as
	// a write-only (!!) attribute).
	public Double2D getLoc() {
		return loc;
	}
	
	// orientation methods
	public double orientation2D() {
		return ori;
	}
	
	public double getOrientation() {
		return ori * 180.0 / Math.PI;
	}
	
	public void setOrientation(double orientation) {
		ori = orientation * Math.PI / 180.0;
	}
	
	public void randomiseOrientation() {
		ori = sim.random.nextDouble() * 2.0 * Math.PI;
	}
	
	// direction towards entity or location
	public void setDirection(Double2D location) {
		Double2D v = fields[0].tv(location, loc);
		ori = Math.atan2(v.y, v.x);
	}
	
	public void setDirection(Entity entity) {
		setDirection(entity.loc);
	}
	
	// perform movement
	public void move(double distance) {
		setLoc(new Double2D(loc.x + distance * Math.cos(ori),
				loc.y + distance * Math.sin(ori)));
	}
	
	public void moveTo(Double2D location, double distance) {
		setDirection(location);
		move(distance);
	}
	
	public void moveTo(Entity entity, double distance) {
		moveTo(entity.loc, distance);
	}
	
	public void moveBy(double x, double y) {
		setLoc(new Double2D(loc.x + x, loc.y + y));
	}
	
	public SimplePortrayal2D getPortrayal() {
        // a compass-shaped thingy in white, of length 4
		return new OrientedPortrayal2D(new SimplePortrayal2D(), 0, 4.0, Color.white,
	            OrientedPortrayal2D.SHAPE_COMPASS );
	}
}