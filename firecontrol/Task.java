package sim.app.firecontrol;

import java.util.LinkedList;
import java.util.ArrayList;
import java.util.LinkedHashSet;



import sim.util.Int2D;

/**
 * This class is used to represent a complex task in the world.
 * Practically speaking, a task represents a fire, not a single cell but a
 * group of cells on fire.
 *
 * @author Albani Dario
 * @email albani@dis.uniroma1.it
 *
 */
public class Task{
	public Int2D centroid;
	public double radius; //the utility?
	public int utility;
	public LinkedList<WorldCell> cells;
	public int id;
	public int UAVassigned;
	public LinkedList<Integer> UAVassigned_ID;
	public UAV manager;
	public int uavNeeded;
	// public LinkedList ManagerIDs;
	// public LinkedHashSet<Double> taskBidsQty; // = new LinkedHashSet<Double>();
	// public LinkedHashSet<Integer> taskBidsID; // = new LinkedHashSet<Integer>();
	public LinkedList<Double> taskBidsQty; // = new LinkedHashSet<Double>();
	public LinkedList<Integer> taskBidsID; // = new LinkedHashSet<Integer>();



	public Task(int id, Int2D centroid, int initialRadius){
		this.centroid = centroid;
		this.radius = initialRadius;
		this.cells = new LinkedList<>();
		this.id = id;
		// this.UAVassigned = 1; // Consider the manager as the first one
		this.taskBidsQty = new LinkedList<Double>(); //taskBidsQty;
		this.taskBidsID = new LinkedList<Integer>(); //taskBidsID;
		this.UAVassigned_ID = new LinkedList<Integer>(); //taskBidsID;
		// this.ManagerIDs = new LinkedList<Integer>();
		this.uavNeeded = uavNeeded;


	}

	public void addCell(WorldCell cell){
		this.cells.add(cell);
	}

	/*
	 * Used to keep the information about the task up to date.
	 * When a new fire is created, the cell calls  this function to let the
	 * task recompute its radius.
	 *
	 * @return true, if the update succeeds and the cell is added
	 */
	public boolean notifyNewFire(WorldCell cell){
		for(WorldCell wc : this.cells){
			if(cell.isNeighborOf(wc)){
				this.cells.add(cell);
				//now update the radius
				Int2D cellPos = new Int2D(cell.x, cell.y);
				this.radius = Math.max(cellPos.distance(this.centroid), this.radius);
				this.utility = this.cells.size(); // add info about a new cell in fire
				return true;
			}
		}
		return false;
	}

	/*
	 * Used to keep the information about the task up to date.
	 * When a new fire is created, the cell calls  this function to let the
	 * task recompute its radius.
	 *
	 * @return true, if the update succeeds and the cell is added
	 */
	public boolean notifyExtinguishedFire(WorldCell cell) {
		if(this.cells.remove(cell)){
			Int2D pos = new Int2D(cell.x, cell.y);
			this.utility -= this.cells.size(); // remove info of the cell from fire
			//if it was a border cell
			if(radius == pos.distance(centroid)){
				//update the radius
				radius = 0;
				for(WorldCell wc : this.cells){
					pos = new Int2D(wc.x, wc.y);
					this.radius = Math.max(pos.distance(centroid), radius);
				}
			}
			return true;
		}
		return false;
	}

	// function to pick a manager closest to the fire
	public void pickManager(Ignite ignite) {
		double closest = 3600; //maximum distance in this size world
		ArrayList<Integer> uav_id_close = new ArrayList<Integer>();

		for (UAV uav : ignite.UAVs) {
			double distance2fire = Math.sqrt(Math.pow(uav.x - this.centroid.x, 2) + Math.pow(uav.y - this.centroid.y, 2));

			if (distance2fire < closest && uav.action == null ) {
				closest = distance2fire;
				uav_id_close.add((int) uav.id); // add the closest uav id to list (the last item will be the closest)
			}
		}
		for (UAV uav : ignite.UAVs) {
			if (uav_id_close.get(uav_id_close.size() -1) == uav.id){ // if it's the last, set as manager
				uav.action = AgentAction.MANAGER;
				this.manager = uav;
			}}


	}

	@Override
	public boolean equals(Object obj){
		Task task = (Task) obj;
		return task.centroid == this.centroid;
	}
}
