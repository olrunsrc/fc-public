/**
 * Implemenation of Simulation2D.
 * 
 * Please keep the API and its documentation in mason_doc.py in sync!!!
 */

package platform_files.MASON;

// java stuff
import java.util.*;

// MASON imports
import ec.util.MersenneTwisterFast;
import sim.engine.*;
import sim.field.continuous.Continuous2D;

// a simulation based on 2D continuous sparse fields
public class Simulation2D extends SimState {

	public double width = 0.0;

	public double height = 0.0;

	public HashMap fields = new HashMap();

	// all the different constructors
	public Simulation2D(double w, double h, String[] fields) {
		super(new MersenneTwisterFast(), new Schedule());
		width = w;
		height = h;
		createFields(fields);
	}

	public Simulation2D(double w, double h, String[] fields, int randomSeed) {
		super(new MersenneTwisterFast(randomSeed), new Schedule());
		width = w;
		height = h;
		createFields(fields);
	}

	// add fields to the simulation
	public void createFields(String[] fieldNames) {
		double bs = bucketSize();
		for (int i = 0; i < fieldNames.length; i++)
			fields.put(fieldNames[i], new Continuous2D(bs, width, height));
	}

	public void createField(String fieldName) {
		double bs = bucketSize();
		fields.put(fieldName, new Continuous2D(bs, width, height));
	}

	// determine the bucket size for the field
	private double bucketSize() {
		// HACK WARANING: The bucket size is crucial for the performance of
		// neighbourhood lookup. As we currently don't use the neigbourhood
		// of a field, it doesn't really matter in our case. If the neighbour-
		// hood is used, this has to be changed.
		// Currently we aim for a minimum of 40 buckets in each direction
		if (width > height) {
			return height / 40.0;
		} else {
			return width / 40.0;
		}
	}

	// called by MASON control upon start of the simulation
	public void start() {
		super.start();

		// make sure that all the fields are empty upon start of the simulation
		Iterator fieldsIter = fields.values().iterator();
		while (fieldsIter.hasNext()) {
			Continuous2D field = (Continuous2D) fieldsIter.next();
			field.clear();
		}
		// populate the fields
		addEntities();
	}

	// needs to be overridden to add entities / agents to the fields
	public void addEntities() {
		// don't do anything here
		// needs to be overridden
	}

	// returns fields by name
	public Continuous2D getField(String fieldName) {
		return (Continuous2D) fields.get(fieldName);
	}

	public Continuous2D[] getFields(String[] fieldNames) {
		Continuous2D[] fieldObjs = new Continuous2D[fieldNames.length];
		for (int i = 0; i < fieldNames.length; i++)
			fieldObjs[i] = (Continuous2D) fields.get(fieldNames[i]);
		return fieldObjs;
	}

	public Continuous2D[] getFields() {
		return (Continuous2D[]) fields.values().toArray(new Continuous2D[0]);
	}
}
