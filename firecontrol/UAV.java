/**
 * A single UAV running over the simulation.
 * This class implements the class Steppable, the latter requires the implementation
 * of one crucial method: step(SimState).
 * Please refer to Mason documentation for further details about the step method and how the simulation
 * loop is working.
 *
 * @author dario albani
 * @mail albani@dis.uniroma1.it
 * @thanks Sean Luke
 */
package sim.app.firecontrol;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.LinkedHashMap;
import java.util.HashSet;
import sim.engine.SimState;
import java.util.Random;
import sim.engine.Steppable;
import sim.util.Double3D;
import sim.util.Int3D;
import java.util.Collections;
import java.util.Comparator;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import sim.field.grid.ObjectGrid2D;
import java.util.Map;
import java.util.HashMap;
import java.lang.Math;

public class UAV implements Steppable{
	private static final long serialVersionUID = 1L;

	// Agent's variable
	public int id; //unique ID
	public double x; //x position in the world
	public double y; //y position in the world
	public double z; //z position in the world
	public Double3D target; //UAV target
	public AgentAction action; //last action executed by the UAV
	public static double communicationRange = 3; //communication range for the UAVs

	// Agent's local knowledge
	public Set<WorldCell> knownCells;
	public Task myTask;

	// Agent's settings - static because they have to be the same for all the
	// UAV in the simulation. If you change it once, you change it for all the UAV.
	public static double linearvelocity = 0.02;

	//used to count the steps needed to extinguish a fire in a location
	public static int stepToExtinguish = 10;
	//used to remember when first started to extinguish at current location
	private int startedToExtinguishAt = -1;

	// Other variables
	public DataPacket data;
	public LinkedList<DataPacket> proposals = new LinkedList<DataPacket>();
	public LinkedList<DataPacket> tasksReceived = new LinkedList<DataPacket>();
	private Lock lock = new ReentrantLock();
	public ObjectGrid2D knownForest;
	public int attempt;
	public static int height = 60; //size of the forest
	public static int width = 60; //size of the forest
	Random random = new Random();
	public int remain_maxID;


	public UAV(int id, Double3D myPosition){
		//set agent's id
		this.id = id;
		//set agent's position
		this.x = myPosition.x;
		this.y = myPosition.y;
		this.z = myPosition.z;
		this.attempt = 0;
		//at the beginning agents have no action
		this.action = null;
		this.knownForest = new ObjectGrid2D(width, height);
		//at the beginning agents have no known cells
		this.knownCells = new LinkedHashSet<>();

		this.proposals = new LinkedList<DataPacket>();
		this.tasksReceived = 	new LinkedList<DataPacket>(); //public Map<UAV, Double> uavTasks;
	}

	// DO NOT REMOVE
	// Getters and setters are used to display information in the inspectors
	public int getId(){
		return this.id;
	}

	public void setId(int id){
		this.id = id;
	}

	public double getX(){
		return this.x;
	}

	public void setX(double x){
		this.x = x;
	}

	public double getY(){
		return this.y;
	}

	public void setY(double y){
		this.y = y;
	}

	public double getZ(){
		return this.z;
	}

	public void setZ(double z){
		this.z = z;
	}

	/**
	 *  Do one step.
	 *  Core of the simulation.
	 */
	public void step(SimState state){
		Ignite ignite = (Ignite)state;

		//select the next action for the agent
		AgentAction a = nextAction(ignite);
		assignTasks(ignite);

		switch(a){
		case SELECT_TASK:
			// ------------------------------------------------------------------------
			// this is where your task allocation logic has to go.
			// be careful, agents have their own knowledge about already explored cells, take that
			// in consideration if you want to implement an efficient strategy.
			// TODO Implement here your task allocation strategy
			//System.err.println("TODO: and now? Use one of methods for tasks assignment!");

			selectTask(ignite); //<- change the signature if needed

			break;

		case SELECT_CELL:
			// ------------------------------------------------------------------------
			// this is where your random walk or intra-task allocation logic has to go.
			// be careful, agents have their own knowledge about already explored cells, take that
			// in consideration if you want to implement an efficient strategy.
			// TODO Implement here your random walk or intra-task allocation strategy
			//System.err.println("TODO: and now? Use random walk or task assignment!");

			selectCell(ignite); //<- change the signature if needed
			break;

		case MOVE:
			move(state);
			break;

		case EXTINGUISH:
			//if true set the cell to be normal and foamed
			if(extinguish(ignite)){
				//retrieve discrete location of this
				// Int3D dLoc = ignite.air.discretize(new Double3D(this.x, this.y, this.z));
				Double3D dLoc = new Double3D(this.x, this.y, this.z);
				//extinguish the fire
				if(ignite.isInBounds(new Double3D((int) dLoc.x, (int) dLoc.y, 1))==true){
					((WorldCell)ignite.forest.field[(int) dLoc.x][(int)dLoc.y]).extinguish(ignite);
					this.target=null;}

				// Check if tasks do not exist anymore -- if so, eliminate all info in uav, tasks, and fires
				for (Task task : ignite.tasks){
					if (task.cells.size()==0 && ignite.tasks.size() > 0){ //|| task.radius == 0

						int indTRem = ignite.tasks.indexOf(task);
						Task Task2Remove = ignite.tasks.get(indTRem);
						ignite.ReselectTask = true;
						ignite.fires -= 1;
						ignite.NumManagersAssigned -= 1;


						// remove info about task from assigned UAVs and add to uav remining to be assigned
						for (int leftID : Task2Remove.UAVassigned_ID){
							if (ignite.uavIDremaining.contains(leftID) == false){
								ignite.uavIDremaining.add(leftID);
								ignite.totalAssigned -= 1;}
							else if (ignite.uavIDassigned.contains(leftID) == true){
								ignite.uavIDassigned.remove(leftID);}

							for (UAV uav : ignite.UAVs ){
								if (uav.id == leftID ){
									uav.myTask = null;
									uav.target = null;
									uav.action = null;}
								else continue;}
							}


						ignite.tasks.remove(Task2Remove); //remove from task list

						System.err.println("Fire has been extinguished with task ID: " + Task2Remove.id + " -------- ");
						System.err.println("UAV that are now available without task: " + ignite.uavIDremaining + " -------- \n");
						break;
					}
				// }}
				// check if termination of job after extinguishing all fires
				else if (ignite.tasks.size()==0  && ignite.fires == 0 ){ //&& ignite.uavIDremaining.size()== ignite.numUAVs
					System.err.println(" Great job! Successful termination. \n");
					// System.exit(0);
				} // successful termination
			// }
		}}
			// this.action = a;
			break;

		default:
			System.exit(0);
			// System.exit(-1);
		}
	}

	/**
	 * What to do next?
	 * TODO Feel free to modify this at your own will in case you have a better
	 * strategy
	 */
	private AgentAction nextAction(Ignite ignite){
		//if I do not have a task I need to take one
		if(this.myTask == null){
			return AgentAction.SELECT_TASK;
		}
		//else, if I have a task but I do not have target I need to take one
		else if(this.target == null){
			return AgentAction.SELECT_CELL;
		}
		//else, if I have a target and task I need to move toward the target
		//check if I am over the target and in that case execute the right action;
		//if not, continue to move toward the target
		// else if(this.target.equals(ignite.air.discretize(new Double3D(x, y, z)))){
		else if(this.target.equals(new Double3D(x, y, z)) && (ignite.isInBounds(new Double3D((int) x, (int) y, 1)))){
			//if on fire then extinguish, otherwise move on
			WorldCell cell = (WorldCell)ignite.forest.field[(int) x][(int) y];
			//store the knowledge for efficient selection
			this.knownCells.add(cell);
			// this.knownForest.field[cell.getX()][cell.getY()] = cell;

			//TODO maybe, you can share the knowledge about the just extinguished cell here!
			// share knowledge of extinguished cell with other UAVs
			Double3D position = new Double3D(x, y, z);
			DataPacket data = new DataPacket(id, position, this.myTask, this.knownCells, false, -1); //note this difference with false for proposedTask

			if(cell.type.equals(CellType.FIRE)){
				return AgentAction.EXTINGUISH;
			}
			else {
				if (cell.type.equals(CellType.EXTINGUISHED)){
					// System.err.println("Stopped a fire at location " + position);
					SendReceiveData(data, ignite, false);
					this.target = null;}
				return AgentAction.SELECT_CELL;
			}
		}
		else{
			return AgentAction.MOVE;
		}
	}

	/**
	 * Take the centroid of the fire and its expected radius and extract the new
	 * task for the agent.
	 */

	private void selectTask(Ignite ignite) { //, LinkedList datapacketReceived_offer

		// Check if UAV is the manager and send task proposal to all UAVs
		for(Task task : ignite.tasks){
			if(this.id == task.manager.id && ignite.NumManagersAssigned < ignite.fires){
				ignite.NumManagersAssigned += 1;
				task.UAVassigned += 1;
				task.UAVassigned_ID.add(this.id);
				ignite.ManagerIDs.add(this.id);


				System.err.println("Manager UAV " + this.id + ": Sent request for proposal to agents.");
				requestProposals(task, ignite);
				// set the action of the manager
				this.action = AgentAction.PROPOSED;
				this.myTask = task;
				this.target = new Double3D(task.centroid.x, task.centroid.y, this.z);
				ignite.uavIDassigned.add(this.id);
				int indexRem = ignite.uavIDremaining.indexOf(this.id);
				if (ignite.uavIDremaining.contains(this.id) == true){
					ignite.uavIDremaining.remove(indexRem);}

				break;
			}}


		// UAVs reply to managers sending their proposal bid
		if (ignite.tasks.size()>1 ){ //newTask == null &&

			// for each requestProposals, UAV calculates bid and proposes to  each manager
			for (DataPacket datapack : this.tasksReceived){
				int taskID = datapack.payload.task.id;
				for (Task task : ignite.tasks){
					if (task.id == taskID){
						if (datapack.header.proposedTask == true && ignite.NumManagersAssigned == ignite.fires && this.tasksReceived.size() == ignite.fires && task.taskBidsID.contains(this.id) == false && ignite.ManagerIDs.contains(this.id) == false){
							datapack.header.id = this.id;
							datapack.payload.position = new Double3D(this.x, this.y, this.z);
							proposeBid(datapack, ignite);
			}}}
		}}
	}



	// Assign task for the drones, used only by the MANAGER
	private void assignTasks(Ignite ignite) {

		// get info about fires
		int totalFire = 0;
		for(Task task : ignite.tasks){
			if (task.radius != 0){
				totalFire += task.utility; // info about cells on fire
			}}

		// Get offers from each UAV to manager and set each's task
		for(Task task : ignite.tasks){
			if(this.id == task.manager.id && ignite.NumManagersAssigned == ignite.fires && task.taskBidsID.size() == (ignite.numUAVs - ignite.fires)){ //&& task.taskBidsID.size() >= (ignite.numUAVs/ignite.fires /task.uavNeeded
				System.err.println("Amount of offers received to Manager " + this.id + " is " + task.taskBidsID.size());

				// calculate the amount of UAVs needed to stop the fire
				task.uavNeeded = (int)(this.myTask.utility * ignite.numUAVs / totalFire);
				// System.err.println("uavNeeded "+ task.uavNeeded);

				System.err.println("For Manager ----- "+ task.manager.id + " needs "+ task.uavNeeded + "UAVs" +", ID of Tasks "+ task.taskBidsID);

				// remove UAVs that already have an action defined
				for (UAV uav : ignite.UAVs){

					if (uav.myTask != null && ignite.ManagerIDs.contains(uav.id) == false){
						int indexValue = task.taskBidsID.indexOf(uav.id);
						double QtyValue = task.taskBidsQty.get(indexValue);
						task.taskBidsID.remove(Integer.valueOf(uav.id));
						task.taskBidsQty.remove(QtyValue);

					}
				}

				// get the #uavNeeded highest offers
				for (int numNeed=0;  numNeed < (task.uavNeeded -1); numNeed++){
					double highestBid = 0;
					for (double BidsQty : task.taskBidsQty){
						if (BidsQty > highestBid){
							highestBid = BidsQty;
						}}
					int indexValue = task.taskBidsQty.indexOf(highestBid);
					int maxID = task.taskBidsID.get(indexValue);
					// remove from the list to keep getting the #uavNeeded amount
					task.taskBidsID.remove(Integer.valueOf((maxID)));
					task.taskBidsQty.remove(highestBid);
					if (numNeed == (task.uavNeeded -1)){
						numNeed += 1;}

					// set the action for those UAVs
					double increaseRadius = 0.5;
					for (UAV uav : ignite.UAVs){
						increaseRadius += 0.7;
						if (uav.id == maxID && task.uavNeeded > task.UAVassigned ){
							uav.myTask = task;
							uav.action = AgentAction.PROPOSED;
							task.UAVassigned += 1;
							task.UAVassigned_ID.add(uav.id);
							ignite.totalAssigned += 1;
							ignite.uavIDassigned.add(uav.id);
							int indexRem = ignite.uavIDremaining.indexOf(uav.id);
							if (ignite.uavIDremaining.contains(uav.id)==true){
								ignite.uavIDremaining.remove(indexRem);}

							// target to be radius of fire closest to the uav
							double radiusAdded = uav.myTask.radius+increaseRadius; // to account for time it takes uav to reach and it has expanded
							double angle = Math.atan2((uav.y - uav.myTask.centroid.y ), (uav.x - uav.myTask.centroid.x));
							double xx = uav.myTask.centroid.x + (radiusAdded * Math.cos(angle));
							double yy = uav.myTask.centroid.y + (radiusAdded * Math.sin(angle));

							uav.target = new Double3D(xx, yy, uav.z);
							System.err.println("UAV " + uav.id + ":" + "\tAssigned task " + uav.myTask.id + "\t by UAV " +
								uav.myTask.manager.id + "\tProposals size " + uav.proposals.size());
						}
					}
					if (task.uavNeeded == task.UAVassigned){
						ignite.TasksCompletedQty += 1;}
					if (ignite.TasksCompletedQty == ignite.fires){
						ignite.Round2on = true;}
					}
				}
			}

			// remove UAVs that already have an action defined
			for(Task task : ignite.tasks){
				for (UAV uav : ignite.UAVs){
					if (ignite.Round2on == true && uav.action != null && task.UAVassigned >= task.uavNeeded && task.taskBidsID.contains(uav.id)==true && ignite.uavIDassigned.contains(uav.id) == true){

						int indexValue = task.taskBidsID.indexOf(uav.id);
						double QtyValue = task.taskBidsQty.get(indexValue);
						task.taskBidsID.remove(Integer.valueOf(uav.id));
						task.taskBidsQty.remove(QtyValue);

						break;
					}}}

			// Reset utility if a fire has been extinguished
			if (ignite.ReselectTask == true){
				ignite.remainingtaskIDs.clear();
				ignite.remainingTaskUtility.clear();
				ignite.Remaining_Utility = false;
			}

			// calculate the "offer" for the remaining uavs given its utility and # of assigned uavs
			utilityRemain:
			for (Task task : ignite.tasks){

				if ((ignite.Round2on == true && ignite.remainingtaskIDs.contains(task.id) == false && ignite.remainingtaskIDs.size() != ignite.fires) || (ignite.ReselectTask == true &&  ignite.remainingtaskIDs.contains(task.id) == false)){

					double leftover_utility = task.utility/ task.UAVassigned;
					while (ignite.remainingTaskUtility.contains(leftover_utility)){
						leftover_utility += 0.01;
					}
					ignite.remainingtaskIDs.add(task.id);
					ignite.remainingTaskUtility.add(leftover_utility);
				}
				if (ignite.remainingtaskIDs.size() == ignite.fires){
					ignite.Remaining_Utility = true;
					break utilityRemain;}
			}


			// Assign the left over uavs that have not been assigned given the remining utility
			double highestUtility = 0;
			if ((ignite.NumManagersAssigned == ignite.fires && ignite.remainingtaskIDs.size() == ignite.fires && ignite.Remaining_Utility == true && ignite.Round2on == true) || (ignite.ReselectTask == true && ignite.Remaining_Utility==true)){
				// get highest utility of tasks
				int remainingqty = ignite.remainingtaskIDs.size();
				for (int numNeed=0;  numNeed < remainingqty ; numNeed++){
					LinkedList<Double> copyRemainingTaskUtility = new LinkedList<Double>();
					copyRemainingTaskUtility = (LinkedList<Double>) ignite.remainingTaskUtility.clone();
					copyRemainingTaskUtility.sort(Comparator.reverseOrder());
					highestUtility = copyRemainingTaskUtility.get(numNeed);
					int indexValue = ignite.remainingTaskUtility.indexOf(highestUtility);
					remain_maxID = ignite.remainingtaskIDs.get(indexValue);

					// assign a uav to this task with the highest remaining utility
					double increaseRadius = 0;
					assign:
					for (UAV uav : ignite.UAVs){
						increaseRadius += 2;
						assignInner:
						for (Task task : ignite.tasks){
							if (uav.action != null){break assignInner;}
							else if (task.id != remain_maxID){break;}
							else if ((task.id == remain_maxID && task.UAVassigned > 0 && task.UAVassigned >= task.uavNeeded && ignite.uavIDremaining.contains(uav.id)==true && uav.action == null && ignite.ManagerIDs.contains(uav.id) == false && ignite.Remaining_Utility == true && ignite.Round2on == true) ||
							(ignite.ReselectTask == true && ignite.Remaining_Utility==true && task.id == remain_maxID && ignite.uavIDremaining.contains(uav.id)==true && uav.action == null)){
								System.err.println("UAV ID remaining to select task " + ignite.uavIDremaining);
								uav.myTask = task;
								uav.action = AgentAction.PROPOSED;
								task.UAVassigned += 1;
								task.UAVassigned_ID.add(uav.id);
								ignite.totalAssigned += 1;
								ignite.uavIDassigned.add(uav.id);
								int indexRem = ignite.uavIDremaining.indexOf(uav.id);
								if (ignite.uavIDremaining.contains(uav.id)==true){
									ignite.uavIDremaining.remove(indexRem);}

								double radiusAdded = uav.myTask.radius+increaseRadius; // to account for time it takes uav to reach and it has expanded
								double angle = Math.atan2((uav.y - uav.myTask.centroid.y ), (uav.x - uav.myTask.centroid.x));
								double xx = uav.myTask.centroid.x + (radiusAdded * Math.cos(angle));
								double yy = uav.myTask.centroid.y + (radiusAdded * Math.sin(angle));


								uav.target = new Double3D(xx, yy, uav.z);
								System.err.println("Remaining UAV " + uav.id + ": " + "\tAssigned task " + uav.myTask.id + "\t by UAV " +
									uav.myTask.manager.id + "\tProposals size " + uav.proposals.size());

								break assign;
				}}}
				}}


			// remove UAVs that already have an action defined
			for(Task task : ignite.tasks){
				for (UAV uav : ignite.UAVs){
					if ((ignite.uavIDremaining.size()== 0 && uav.action != null && task.UAVassigned >= task.uavNeeded && task.taskBidsID.contains(uav.id)==true && ignite.uavIDassigned.contains(uav.id) == true) || (ignite.ReselectTask == true && ignite.Remaining_Utility==true && task.taskBidsID.contains(uav.id)==true)){

						int indexValue = task.taskBidsID.indexOf(uav.id);
						double QtyValue = task.taskBidsQty.get(indexValue);
						task.taskBidsID.remove(Integer.valueOf(uav.id));
						task.taskBidsQty.remove(QtyValue);
					}}
			}
		}


	/**
	 * Take the centroid of the fire and its expected radius and select the next
	 * cell that requires closer inspection or/and foam.
	 */
	 private void selectCell(Ignite ignite) {
		//remember to set the new target at the end of the procedure
		// Double3D newTarget = null;

		// TODO - done


		// Do random walk
		// Select random cell near the cell where the UAV is located
		if (this.myTask != null ){ //&& this.target == null

			int randX, randY;
			int tries = 0;
			double radius = myTask.radius;
			Double3D newCellTarget;
			WorldCell cell;
			boolean inBoundCell = false;
			boolean inFire = false;

			selectCellLoop:
			do{
				randX = random.nextInt((1 + 1) + 1) - 1;
				randY = random.nextInt((1 + 1) +1) - 1;

				newCellTarget = new Double3D(this.x + randX, this.y + randY, this.z);

				if(ignite.isInBounds(new Double3D((int) newCellTarget.x, (int) newCellTarget.y, 1))==true){
					cell = (WorldCell)ignite.forest.field[(int) newCellTarget.x][(int) newCellTarget.y];
					inBoundCell = true;
					inFire = cell.type.equals(CellType.FIRE);
				}

				tries += 1;

				selectCellInner:
				if(tries >= 4 && tries < 10){ //
					int n = tries-2;
					int randXX = random.nextInt((n + n) + n) - n;
					int randYY = random.nextInt((n + n) + n) - n;
					newCellTarget = new Double3D(this.myTask.centroid.x + randXX, this.myTask.centroid.y+ randYY, this.z);

					if(ignite.isInBounds(new Double3D((int) newCellTarget.x, (int) newCellTarget.y, 1))==true){
						cell = (WorldCell)ignite.forest.field[(int) newCellTarget.x][(int) newCellTarget.y];
						inBoundCell = true;
						inFire = cell.type.equals(CellType.FIRE);
						// break;// selectCellLoop;
					}
					else {
						inBoundCell = false;
						// break;
					}
				}
				else if (tries >= 10) {
					tries = 0;
					break selectCellInner;} //selectCellLoop;}
				}
			while((randX == 0 && randY == 0) || inBoundCell==false || inFire == false);

			if (tries >= 10) {

				newCellTarget = new Double3D(this.myTask.centroid.x, this.myTask.centroid.y, this.z);
				this.target = newCellTarget;
			}
			else {
				this.target = newCellTarget;
			}
		}
	}

// ***************** STRATEGY 2 -- DOES NOT WORK AS EFFICIENTLY ******************
// 	if(ignite.isInBounds(new Double3D((int) newCellTarget.x, (int) newCellTarget.y, 1))==true){
// 		cell = (WorldCell)ignite.forest.field[(int) newCellTarget.x][(int) newCellTarget.y];
// 		inBoundCell = true;
// 		inFire = cell.type.equals(CellType.FIRE);
// 		// System.err.println("yes inbound and in fire? " + inFire);
// 	}
// 	tries += 1;
// 	// System.err.println("UAV: aqui 2 2 2 and tries: " + tries);
//
// 	selectCellInner:
// 	if(tries >= 4 && tries < 10){ //
// 		tries += 1;
// 		// System.err.println("tries " + tries);
// 		int n = tries - 2;
// 		int randXX = random.nextInt((n + n) + n) - n;
// 		int randYY = random.nextInt((n + n) + n) - n;
// 		newCellTarget = new Double3D(this.myTask.centroid.x + randXX, this.myTask.centroid.y+ randYY, this.z);
//
//
// 		if(ignite.isInBounds(new Double3D((int) newCellTarget.x, (int) newCellTarget.y, 1))==true){
// 			cell = (WorldCell)ignite.forest.field[(int) newCellTarget.x][(int) newCellTarget.y];
// 			inBoundCell = true;
// 			inFire = cell.type.equals(CellType.FIRE);
// 			// System.err.println("UAV: aqui after " + newCellTarget.x + ", " + newCellTarget.y);
//
// 			// break;// selectCellLoop;
// 		}
// 		else {
// 			inBoundCell = false;
// 			break;
// 		}
// 	}
// 		// this.attempt += 1;
// 	else if (tries >= 10) {
// 		tries = 0;
// 		// break selectCellInner;}
// 		break selectCellLoop;}
// 	// System.err.println("randX " + randX+ "randY " + randX);
// 	}
// 	// || !(ignite.isInBounds(newCellTarget))
// while((randX == 0 && randY == 0) || inBoundCell==false || inFire == false);
//
// if (tries >= 15) {
// 	int n = 5;
// 	int randXX = random.nextInt((n + n) + n) - n;
// 	int randYY = random.nextInt((n + n) + n) - n;
// 	// Task newTask = tasks.get(random.nextInt(tasks.size()));
// 	// newCellTarget = new Double3D(newTask.centroid.x, newTask.centroid.y, this.z);
// 	newCellTarget = new Double3D(this.myTask.centroid.x + randXX , this.myTask.centroid.y + randYY, this.z);
// 	this.target = newCellTarget;
// 	// this.myTask = null;
// }
// else {
// 	// System.err.println("new target \n");
// 	this.target = newCellTarget;
// }
// }
// }


	/**
	 * Move the agent toward the target position
	 * The agent moves at a fixed given velocity
	 * @see this.linearvelocity
	 */
	public void move(SimState state){
		Ignite ignite = (Ignite) state;

		// retrieve the location of this
		Double3D location = ignite.air.getObjectLocationAsDouble3D(this);
		double myx = location.x;
		double myy = location.y;
		double myz = location.z;

		// compute the distance w.r.t. the target
		// the z axis is only used when entering or leaving an area
		double xdistance = this.target.x - myx;
		double ydistance = this.target.y - myy;

		if(xdistance < 0)
			myx -= Math.min(Math.abs(xdistance), linearvelocity);
		else
			myx += Math.min(xdistance, linearvelocity);

		if(ydistance < 0){
			myy -= Math.min(Math.abs(ydistance), linearvelocity);
		}
		else{
			myy += Math.min(ydistance, linearvelocity);
		}

		// update position in the simulation
		ignite.air.setObjectLocation(this, new Double3D(myx, myy, myz));
		// update position local position
		this.x = myx;
		this.y = myy;
		this.z = myz;
	}

	/**
	 * Start to extinguish the fire at current location.
	 * @return true if enough time has passed and the fire is gone, false otherwise
	 * @see this.stepToExtinguish
	 * @see this.startedToExtinguishAt
	 */
	private boolean extinguish(Ignite ignite){
		if(startedToExtinguishAt==-1){
			this.startedToExtinguishAt = (int) ignite.schedule.getSteps();
		}
		//enough time has passed, the fire is gone
		if(ignite.schedule.getSteps() - startedToExtinguishAt == stepToExtinguish){
			startedToExtinguishAt = -1;
			return true;
		}
		return false;
	}

	/**
	 * COMMUNICATION
	 * Check if the input location is within communication range
	 */
	public boolean isInCommunicationRange(Double3D otherLoc){
		Double3D myLoc = new Double3D(x,y,z);
		boolean close = false;
		if (otherLoc.x <= UAV.communicationRange || otherLoc.y <= UAV.communicationRange || otherLoc.z <= UAV.communicationRange){
			close = true;
		}
		return close;
	}

	/**
	 * COMMUNICATION
	 * Send a message to the team
	 */

	public void SendReceiveData(DataPacket packet, Ignite ignite, boolean proposing){
		//TODO -- done

		boolean reached_manager = false;
		boolean reached_allUAVs = false;
		Integer firstTime = 0;
		Double3D start_loc;
		LinkedList<Integer> uavReceived = new LinkedList<Integer>();
		LinkedList<Double> uavReceived_pos = new LinkedList<Double>();

		// while (reached_manager == false || reached_allUAVs == false) { // until it hasn't arrive to manager, do
		outerloop:
		for (int h=0; h <= (ignite.numUAVs-1); h++){ // for the amount of uavs
			innerloop:
			for (int k=0; k <= (uavReceived.size()); k++){ // for each close uav
				if (h == 0) {start_loc = packet.payload.position;}
				else {start_loc = new Double3D (uavReceived_pos.get(k*4+1), uavReceived_pos.get(k*4+2), uavReceived_pos.get(k*4+3) );} // set the new start location as the one for close UAV

				for (UAV uav : ignite.UAVs) {

						DataPacket uav_datapgk = uav.data;
						Double3D distance2uav = new Double3D (Math.abs((uav.x - start_loc.x)), Math.abs((uav.y - start_loc.y)), Math.abs((uav.z - start_loc.z)));

						if (isInCommunicationRange(distance2uav) == true && uavReceived.contains(uav.id) == false) { //if within the distance of communication range: && uavReceived.contains(uav.id) == false

							if (proposing == false) {
								uavReceived.add(uav.id); // add the id of the uav that reveived the packet
								uavReceived_pos.add((double) uav.id); // add the id of the uav that reveived the packet
								uavReceived_pos.add((double) uav.x); // add the x of the uav that reveived the packet
								uavReceived_pos.add((double) uav.y); // add the x of the uav that reveived the packet
								uavReceived_pos.add((double) uav.z); // add the x of the uav that reveived the packet

								// add info to manager proposal data
								if (packet.header.offerBid >= 0.0 ){
									// Add info of proposal to Manager
									uav.proposals.add(packet);

									// Add to tasks received
									for(Task task : ignite.tasks){
										if (task.id == packet.payload.task.id ){

												if (task.taskBidsID.contains(packet.header.id) == true){
													int indexDel = task.taskBidsID.indexOf(packet.header.id);
													double offerDel = task.taskBidsQty.get(indexDel);
													task.taskBidsQty.remove(offerDel);
													task.taskBidsID.remove(Integer.valueOf(packet.header.id));
												}

											task.taskBidsID.add(packet.header.id);
											task.taskBidsQty.add(packet.header.offerBid);
										}}

									if (uavReceived.contains(packet.payload.task.manager.id)==true){
										System.err.println("Data from UAV ID: "+ packet.header.id + " reached task Manager with ID: " + packet.payload.task.manager.id + " for task ID " + packet.payload.task.id + " \n" ); // problem is

										reached_manager = true;
										break outerloop;}
								}
								else if (packet.header.offerBid >= 0.0 && packet.payload.task.manager.id == packet.header.id){
									break outerloop;}

								else if (packet.header.offerBid < 0.0 && uavReceived.contains(packet.payload.task.manager.id)==true){
									break outerloop;}
							}

				// ------------------------------------------------------------------ //

							// sending request for proposals from Manager to UAVs
							else if (proposing == true){

								uavReceived.add(uav.id); // add the id of the uav that reveived the packet
								uavReceived_pos.add((double) uav.id); // add the id of the uav that reveived the packet
								uavReceived_pos.add((double) uav.x); // add the x of the uav that reveived the packet
								uavReceived_pos.add((double) uav.y); // add the y of the uav that reveived the packet
								uavReceived_pos.add((double) uav.z); // add the z of the uav that reveived the packet

								uav.tasksReceived.add(packet); // add request for proposal to uav

								Integer included_count = 0;
								for ( int n=0; n <= ignite.numUAVs; n++ ) { // check that all UAV has received data
									if (uavReceived.contains(n) == true) {
										included_count++;}}
								if (included_count == ignite.numUAVs) {
										System.err.println("Data from Manager ID: " + packet.payload.task.manager.id + " reached all UAVs \n");
										reached_allUAVs = true;
										break outerloop;}
						}}}}
			if (reached_allUAVs == true || reached_manager == true) break;
		}}


	/**
	 * COMMUNICATION
	 * Retrieve the status of all the agents in the communication range.
	 * @return an array of size Ignite.tasks().size+1 where at position i you have
	 * the number of agents enrolled in task i (i.e. Ignite.tasks().get(i)).
	 *
	 * HINT: you can easily assume that the number of uncommitted agents is equal to:
	 * Ignite.numUAVs - sum of all i in the returned array
	 */

	// Request for bid
	public void requestProposals(Task task, Ignite ignite){

		Double3D position = new Double3D(task.centroid.x,task.centroid.y,this.z);
		DataPacket proposedTask = new DataPacket(this.id, position, task, null, true, -1);
		SendReceiveData(proposedTask, ignite, true);
	}

	// Accept task
	public void proposeBid(DataPacket dataTask, Ignite ignite){
		Double3D position_uav = dataTask.payload.position;
		double distance = Math.sqrt(Math.pow(position_uav.x - dataTask.payload.task.centroid.x,2) + Math.pow(position_uav.y - dataTask.payload.task.centroid.y,2));
		double offer = dataTask.payload.task.utility / distance;

		System.err.println("Offer for task: " + dataTask.payload.task.id + " from UAV" + dataTask.header.id + " is: " + offer + " \t for manager " + dataTask.payload.task.manager.id);
		DataPacket offerBid = new DataPacket(dataTask.header.id, position_uav, dataTask.payload.task, null, false, offer);
		SendReceiveData(offerBid, ignite, false);//}
	}

	// Refuse task
	public void refuse(Task task, Ignite ignite){
		Double3D position_uav = new Double3D(this.x, this.y, this.z);
		DataPacket offerBid = new DataPacket(this.id, position_uav, task, null, false, 0);
		SendReceiveData(offerBid, ignite, false);//}
	}

	@Override
	public boolean equals(Object obj){
		UAV uav = (UAV) obj;
		return uav.id == this.id;
	}

	@Override
	public String toString(){
		return id+"UAV-"+x+","+y+","+z+"-"+action;
	}
}
