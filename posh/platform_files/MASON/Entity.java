/**
 * Implementation of a MASON entity.
 * 
 * Please keep the API and its documentation in mason_doc.py in sync!!!
 */

package platform_files.MASON;

// Java imports
import java.lang.Math;
import java.util.*;
import java.awt.Color;

// MASON imports
import sim.util.*;
import sim.field.continuous.Continuous2D;
import sim.portrayal.*;
import sim.portrayal.simple.*;

public class Entity {

	public Simulation2D sim = null;

	public Continuous2D[] fields = null;

	public Double2D loc = null;

	public double neighbourhoodSize = 40.0;

	// contructors
	public Entity(Simulation2D sim, String[] fieldNames, Double2D loc,
			double neighbourhoodSize) {
		this.sim = sim;
		// get the fields from the simulation
		this.fields = sim.getFields(fieldNames);
		setLoc(loc);
		this.neighbourhoodSize = neighbourhoodSize;
	}

	public Entity(Simulation2D sim, String[] fieldNames, Double2D loc) {
		this.sim = sim;
		this.fields = sim.getFields(fieldNames);
		setLoc(loc);
	}

	public Entity(Simulation2D sim, String[] fieldNames) {
		this.sim = sim;
		this.fields = sim.getFields(fieldNames);
		randomiseLocation();
	}

	public Entity(Simulation2D sim) {
		this.sim = sim;
		this.fields = sim.getFields();
		randomiseLocation();
	}

	// location management methods
	public Double2D getLoc() {
		return loc;
	}

	public void setLoc(Double2D newLoc) {
		// convert them into toroidal coordinates
		Continuous2D field = fields[0];
		loc = new Double2D(field.stx(newLoc.x), field.sty(newLoc.y));
		// set the location on all fields
		for (int f = 0; f < fields.length; f++)
			fields[f].setObjectLocation(this, loc);
	}

	public Double2D getRandomLocation() {
		return new Double2D(sim.random.nextDouble() * sim.width, sim.random
				.nextDouble()
				* sim.height);
	}

	public void randomiseLocation() {
		setLoc(getRandomLocation());
	}

	public double getNeighbourhoodSize() {
		return neighbourhoodSize;
	}

	public void setNeighbourhoodSize(double size) {
		if (size > 0)
			neighbourhoodSize = size;
	}

	// variants of getFields()
	public Continuous2D[] getFields(String[] fieldNames) {
		return sim.getFields(fieldNames);
	}

	public Continuous2D[] getFields(String fieldName) {
		Continuous2D[] field = { sim.getField(fieldName) };
		return field;
	}

	public Continuous2D[] getFields() {
		return fields;
	}

	// variants of setFields()
	public void setFields(String[] fieldNames) {
		// remove object from currently set fields
		for (int f = 0; f < fields.length; f++)
			fields[f].remove(this);
		// put itself on the new fields
		fields = sim.getFields(fieldNames);
		setLoc(loc);
	}

	public void setFields(String fieldName) {
        String[] fieldNames = { fieldName };
		setFields(fieldNames);
	}

	// distance to other entities
	public double distance(Entity e) {
		return Math.sqrt(fields[0].tds(loc, e.loc));
	}

	public double distance(Double2D loc) {
		return Math.sqrt(fields[0].tds(this.loc, loc));
	}

	public Double2D vectorTowards(Entity e) {
		return fields[0].tv(e.loc, loc);
	}

	public Double2D vectorTowards(Double2D loc) {
		return fields[0].tv(loc, this.loc);
	}
	
	// neighbours methods (long live a default argument)
	public Entity[] neighbours(Continuous2D[] fields, double neighbourhoodSize) {
		// as we need dynamic sizing, use an array list for now
		ArrayList entities = new ArrayList();
		// go through all the fields to find the neighbours
		for (int f = 0; f < fields.length; f++) {
			Bag neighbourObjs = fields[f].getObjectsWithinDistance(loc, neighbourhoodSize);
			for (int o = 0; o < neighbourObjs.numObjs; o++) {
				Object obj = neighbourObjs.objs[o];
			    // only add if it is actually within the correct distance
				if ((obj != null) &&
					(obj != (Object) this) &&
					(!entities.contains(obj)) &&
					(distance((Entity) obj) <= neighbourhoodSize))
				    entities.add(obj);
			}
		}
		// convert into standard array
		Entity[] entityArray = new Entity[entities.size()];
		for (int e = 0; e < entities.size(); e++)
			entityArray[e] = (Entity) entities.get(e);
		return entityArray;
	}
	
	public Entity[] neighbours(String[] fieldNames, double neightbourhoodSize) {
		return neighbours(sim.getFields(fieldNames), neighbourhoodSize);
	}
	
	public Entity[] neighbours(String fieldName, double neighbourhoodSize) {
		String[] fieldNames = { fieldName };
		return neighbours(fieldNames, neighbourhoodSize);
	}
	
	public Entity[] neighbours(String[] fieldNames) {
	    return neighbours(fieldNames, neighbourhoodSize);	
	}
	
	public Entity[] neighbours(String fieldName) {
		String[] fieldNames = { fieldName };
		return neighbours(fieldNames);
	}
	
	public Entity[] neighbours() {
	    return neighbours(fields, neighbourhoodSize);	
	}
	
	// closest
	public Entity closest(Continuous2D[] fields, Entity[] exclude) {
		double minDist = (sim.height * sim.height + sim.width * sim.width);
		Entity closestEntity = null;
		for (int f = 0; f < fields.length; f++) {
			Bag fieldObjs = fields[f].getAllObjects();
			for (int o = 0; o < fieldObjs.numObjs; o++) {
				Object obj = fieldObjs.objs[o];
				// ignore the object if it's oneself
				if (obj != (Object) this) {
					double dist = distance((Entity) obj);
					if (dist < minDist) {
						minDist = dist;
						closestEntity = (Entity) obj;
					}
				}
			}
		}
	    return closestEntity;	
	}
	
	public Entity closest(String[] fieldNames, Entity[] exclude) {
		return closest(sim.getFields(fieldNames), exclude);
	}
	
	public Entity closest(String fieldName, Entity[] exclude) {
	    String[] fieldNames = { fieldName };
	    return closest(fieldNames, exclude);
	}
	
	public Entity closest(String[] fieldNames) {
		return closest(fieldNames, new Entity[0]);
	}
	
	public Entity closest(String fieldName) {
		return closest(fieldName, new Entity[0]);
	}
	
	public Entity closest() {
		return closest(fields, new Entity[0]);
	}
	
	// get the width of the field
	public double fieldWidth(Continuous2D[] fields) {
		// as the area is toroidal, determining the width of the field
		// is somehow tricy. Hence, this method does it in a simple way
		// by taking the two agents which are furthest apart and
		// returns their distance. Not that this requires O(N^2) comparisons.
		double maxDist = 0;
		for (int f1 = 0; f1 < fields.length; f1++) {
			Bag field1Objs = fields[f1].getAllObjects();
			for (int o1 = 0; o1 < field1Objs.numObjs; o1++) {
				Object obj1 = field1Objs.objs[o1];
				for (int f2 = 0; f2 < fields.length; f2++) {
					Bag field2Objs = fields[f2].getAllObjects();
					for (int o2 = 0; o2 < field2Objs.numObjs; o2++) {
						Object obj2 = field2Objs.objs[o2];
						double dist = ((Entity) obj1).distance((Entity) obj2);
						if (dist > maxDist)
							maxDist = dist;
					}
				}
			}
		}
		return maxDist;
	}
	
	public double fieldWidth(String[] fieldNames) {
		return fieldWidth(sim.getFields(fieldNames));
	}
	
	public double fieldWidth(String fieldName) {
		String[] fieldNames = { fieldName };
		return fieldWidth(fieldNames);
	}
	
	public double fieldWidth() {
		return fieldWidth(fields);
	}
	
	// average field width
	public double averageFieldWidth(Continuous2D[] fields) {
		int compareCount = 0;
		double avgWidth = 0.0;
		for (int f1 = 0; f1 < fields.length; f1++) {
			Bag field1Objs = fields[f1].getAllObjects();
			for (int o1 = 0; o1 < field1Objs.numObjs; o1++) {
				Object obj1 = field1Objs.objs[o1];
				for (int f2 = 0; f2 < fields.length; f2++) {
					Bag field2Objs = fields[f2].getAllObjects();
				    for (int o2 = 0; o2 < field2Objs.numObjs; o2++) {
				    	Object obj2 = field2Objs.objs[o2];
				    	if (obj1 != obj2) {
				    		avgWidth = ((Entity) obj1).distance((Entity) obj2);
				    		compareCount += 1;
				    	}
				    }
				}
			}
		}
		if (compareCount > 0)
			return avgWidth / compareCount;
		return 0.0;
	}
	
	public double averageFieldWidth(String[] fieldNames) {
		return averageFieldWidth(sim.getFields(fieldNames));
	}
	
	public double averageFieldWidth(String fieldName) {
		String[] fieldNames = { fieldName };
		return averageFieldWidth(fieldNames);
	}
	
	public double averageFieldWidth() {
		return averageFieldWidth(fields);
	}
	
	// fieldCentre method
	public Double2D fieldCentre(Continuous2D[] fields) {
		// the centrepoint between the two entities which are furthest apart
		double maxDist = 0;
		Continuous2D field = fields[0];
		Entity ent1 = null;
		Entity ent2 = null;
		for (int f1 = 0; f1 < fields.length; f1++) {
			Bag field1Objs = fields[f1].getAllObjects();
			for (int o1 = 0; o1 < field1Objs.numObjs; o1++) {
				Object obj1 = field1Objs.objs[o1];
				for (int f2 = 0; f2 < fields.length; f2++) {
					Bag field2Objs = fields[f2].getAllObjects();
					for (int o2 = 0; o2 < field2Objs.numObjs; o2++) {
						Object obj2 = field2Objs.objs[o2];
						if (obj1 != obj2) {
							double dist =
								field.tds(((Entity) obj1).loc, ((Entity) obj2).loc);
							if (dist > maxDist) {
								maxDist = dist;
								ent1 = (Entity) obj1;
								ent2 = (Entity) obj2;
							}
						}
					}
				}
			}
		}
		// ent1 and ent2 are the most 'distant' objects
		if (ent1 == null)
			return new Double2D(0.0, 0.0);
		Double2D v = field.tv(ent2.loc, ent1.loc);
		return new Double2D(field.stx(ent1.loc.x + 0.5 * v.x),
				field.sty(ent1.loc.y + 0.5 * v.y));
	}
	
	public Double2D fieldCentre(String[] fieldNames) {
		return fieldCentre(sim.getFields(fieldNames));
	}
	
	public Double2D fieldCentre(String fieldName) {
		String[] fieldNames = { fieldName };
		return fieldCentre(fieldNames);
	}
	
	public Double2D fieldCentre() {
		return fieldCentre(fields);
	}
	
	// distance to the centre
	public double distanceToCentre(Continuous2D[] fields) {
		return distance(fieldCentre(fields));
	}
	
	public double distanceToCentre(String[] fieldNames) {
		return distanceToCentre(sim.getFields(fieldNames));
	}
	
	public double distanceToCentre(String fieldName) {
		String[] fieldNames = { fieldName };
		return distanceToCentre(fieldNames);
	}
	
	public double distanceToCetnre() {
		return distanceToCentre(fields);
	}
	
	public SimplePortrayal2D getPortrayal() {
		// creates a simple blue rectangle of scale 5
        return new RectanglePortrayal2D( Color.blue, 5 );
	}

}