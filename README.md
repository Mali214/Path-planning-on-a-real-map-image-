# Path-planning-on-a-real-map-image-
This subsection will be discussing the performance of the different planning algorithms will be
compared as well as their properties. A comparison criterion will be presented and testing
environment (MSA university campus) preparation will be shown.

Preparation of the testing map

Before proceeding with testing and comparison between the different algorithms discussed in
earlier there must be an environment (map) prepared for these algorithms. to take a more practical
approach for testing a satellite image of a university campus was chosen to be the planning
environment shown in Figure 4.1.
The image used is not well used to be used directly to be used in a MATLAB simulation for path
planning so some image processing was done to obtain a more usable image. The goal here was to
differentiate between the drivable surface (roads) and the non-drivable surface (curbs, building,
grass, etc.). A standard procedure here is to threshold the image. However, because the roads,
buildings, and curbs have the same range of grayscale the resulting image will lose a lot of
important details, see Figure 4.1, the satellite image has to be tuned before thresholding. To tune
the satellite image, the image was sharpened then the contrast level was increased, a median filter
was applied to smooth out the image to produce the tuned image in Figure 4.3. Finally, the image
was the threshold to mark the drivable surface by 1 and the non-drivable surface by 0 as shown in
Figure 4.4.

![image](https://user-images.githubusercontent.com/122736585/212552391-13d38c32-a76a-4f8f-8db3-6700f7fe00fc.png)

Driving Scenarios

Each planning algorithm will be tested in 5 different scenarios on the map. Note that some driving
scenarios below have starting or ending points in locations filled with obstacles and have little free
space, those are referred to as complex maneuvers.
• Scenario 1. Figure 4.5 shows a short path with no complex maneuvers (almost a straight
line).
• Scenario 2. Figure 4.6 shows a long and simple path with no complex maneuvers.
• Scenario 3. Figure 4.7 shows a medium-length path with two complex maneuvers.
• Scenario 4. Figure 4.8 shows a short path with one complex maneuver.
• Scenario 5. Figure 4.9 shows a long path with two complicated maneuvers. 

![image](https://user-images.githubusercontent.com/122736585/212552446-8f3d6a31-04a5-4603-be33-054080cfa3a0.png)

Simulation results

a) PRM Results
![image](https://user-images.githubusercontent.com/122736585/212552509-5433fc18-ea6e-44f4-a4b4-be79b2da94b8.png)

b)RRT result
![image](https://user-images.githubusercontent.com/122736585/212552549-61af8c9d-7670-453c-9381-6e8377a6cbef.png)

c)RRT * Results
![image](https://user-images.githubusercontent.com/122736585/212552588-cfa3e824-8ec2-48b1-89eb-a82a3ffa63c9.png)

d)A* results
![image](https://user-images.githubusercontent.com/122736585/212552607-ff13965f-f45d-4bd6-bab6-9fe062a460da.png)

e) Hybrid A* Results
![image](https://user-images.githubusercontent.com/122736585/212552629-26131a73-734b-4a6f-b0af-7d63918d0dd9.png)

Total distance comparison for all Scenarios

The results from the simulation listed . show an almost
steady margin between the 5 methods in all five scenarios. In 4 out of the 5 scenarios A* and
Hybrid A* are the two most optimal algorithms, the only different case is in scenario 1 where PRM
actually the shortest path planner. As shown in figures Figure 4.10: Figure 4.24 the least optimal
algorithm with the longest path length is RRT followed by RRT*, the absolute optimal algorithm
in these scenarios is the A*. However, the Hybrid A* is not far off in terms of total path length.

![image](https://user-images.githubusercontent.com/122736585/212552675-1bcbb2bd-f0dd-42db-aa9a-01fc85b55593.png)

Total planning time for all Scenarios

![image](https://user-images.githubusercontent.com/122736585/212552693-48c1198a-93e9-457c-b3e1-ca3729559a94.png)

Path Planning Simulation Results
This subsection will be selecting the algorithm that would mostly be the best among the ones
mentioned in this report to be implemented in an autonomous vehicle for this particular
environment shown in Figure 4.1. Table 4.3 lists the four categories mentioned in subsection
4.2.2.3, on which each algorithm is evaluated, and Error! Reference source not found. Table 4.4
lists a summary of the simulation results. From Table 4.4 it can be seen that all of the algorithms
are complete, only two are optimal and three of them have the same time complexity except the
A* and Hybrid A* this is because their time complexity depends on the heuristic function. As for
memory only two scored a low memory usage, the hybrid A* and PRM.
From the literature survey, Table 4.3 and Table 4.4. it can be concluded that the best algorithm to
be implemented in an autonomous vehicle of these five algorithms is the Hybrid A*. The reasons
behind this conclusion are:
1) It takes the least memory of all algorithms.
2) Its solution approaches the optimal solution most of the time
3) It is well suited to be used with non-holonomic robots (autonomous cars) 
![image](https://user-images.githubusercontent.com/122736585/212552730-ef26c230-2c9b-42f0-ac4f-613281a979ac.png)

![image](https://user-images.githubusercontent.com/122736585/212552755-a3067a2f-dae5-4378-bc9d-13cde4444727.png)





