# Fire_Extinguisher_MAS
Multi-Agent System Project on UAV Fire Extinguisher using Java


Introduction:

An important topic in Artificial Intelligence is the cooperation of agents to perform tasks and achieve a common goal. This project uses a simulation to manage and control wildfires using UAVs (Unmanned Aerial Vehicles) that traverse in the world with a special foam to extinguish the fires before it depletes the entire forest. The simulation and code are developed in Java using the MASON Toolkit. The task allocation protocol for fire depletion give communication constraints is Contract Net Protocol (CNP) and a Random Walk strategy for exploration task.

Project Background: 

The forest world consists of a 2D grid, as seen in Figure 1, that has cells that can have the following characteristics: forest/normal (green), foamed and trees are safe (returns to green), burnt and unrecoverable (gray), on-fire and possibly recoverable (red), lake/water (blue). The UAV drones are characterized as one white cell and can only be aware of the current cell it is over. Due to payload constraints, the UAVs can only communicate at a limited range.

Each fire that is started, is considered as a task with its centroid where it started, the radius describing its extension. UAVs only have this two information and no knowledge of the cells around the centroid. With the use of CNP, agents select a task from the fires available. The task Manager announces the task to all agents, which then reply with an offer determined by their position and the utility of the task. The manager then awards the agents with the highest bids that are still available with the task. The agents then are all assigned to their tasks and their target as they move towards it and start executing the Random Walk (RW) to explore the neighboring cells and stop any cells that are burning. The goal is reached when either all possible cells are extinguished or burned. Since there are multiple managers, groups, and tasks this CNP task is decentralized. 
