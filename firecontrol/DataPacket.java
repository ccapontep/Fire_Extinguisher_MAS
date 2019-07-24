/*
 * Simple structure for a data packet.
 *
 * @author dario albani
 * @mail dario.albani@istc.cnr.it
 */

package sim.app.firecontrol;
import sim.util.Double3D;
import java.text.SimpleDateFormat;
import java.util.Set;
import java.util.Date;


public class DataPacket{

	public class Header{
		public int id; // give id of the proposal
		public String timestamp; // time and date of when proposal is sent
		public boolean proposedTask; // the proposed task
		public double offerBid; // bid offer from UAVs
		public Header(int id, boolean proposedTask, double offerBid){
			//TODO - Done
			//System.err.println("TODO: You have to define the header. Maybe a timestamp and an ID?");
			this.id = id;
			this.timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());
			this.proposedTask = proposedTask;
			this.offerBid = offerBid;
		}
	};

	public class Payload{
		public Double3D position; // position of UAV
		public Task task; // task the UAV is currently doing
		public Set<WorldCell> knownCells; //info of current cell (location and type)
		public Payload(Double3D position, Task task, Set<WorldCell> knownCells){
			//TODO - Done
			//System.err.println("TODO: You have to define the payload. What are you going to share?");
			this.position = position;
			this.task = task;
			this.knownCells = knownCells;
		}
	};

	public Header header;
	public Payload payload;

	//TODO - Done
	//define the data packet according to your payload and your header.
	//please, note that if you do not define a good header you could have problem
	//with duplicates messages
	public DataPacket(int id, Double3D position, Task task, Set<WorldCell> knownCells, boolean proposedTask, double offerBid){
		this.header = new Header(id, proposedTask, offerBid);
		this.payload = new Payload(position, task, knownCells);
	}
}
