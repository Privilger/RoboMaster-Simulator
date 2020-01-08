## rm_simulator
A refer simulating system in AI challenge
### 1.How to run

After catkin_make, open a terminal and run:
``` 
rosrun rm_simulator simulate_refer_system
``` 
### 2.msg

#### cellState #state for each cell (totally 6 cells)


+ uint8[7] state

occupied, cantMove, cantShoot, redHealth, blueHealth, redBullet, blueBullet 

E.g. 0,0,0,1,0,0,0 means that the cell is unoccupied and adding redHealth.


#### referSystemInfo


+ uint64[4]       robotHealth

In the beginning, each robot has 2000 HP.

+ uint64[4]       remain_bullet

In the beginning, only robot 0 and 2 have 50 bullets each.

+ uint8[4]        robotShoot

0: disable or 1: enable -> shoot. In the beginning, [1,1,1,1]

+ float64[4]      robotShootDebuffTime

Record the cannot shoot debuff active time

+ uint8[4]        robotMove

0: disable or 1: enable -> move. In the beginning, [1,1,1,1]

+ float64[4]      robotMoveDebuffTime

Record the cannot move debuff active time

+ CellStatus[6]   cellState

cellState #state for each cell (totally 6 cells)

+ uint64[6]       cellStateNumber

Convey binary number to decimal number.


E.g.

cellState:

  - 
    state: [1, 0, 0, 0, 0, 1, 0]
  - 
    state: [0, 0, 0, 1, 0, 0, 0]
  - 
    state: [0, 0, 1, 0, 0, 0, 0]
  - 
    state: [0, 0, 0, 0, 0, 0, 1]
  - 
    state: [0, 0, 0, 0, 1, 0, 0]
  - 
    state: [0, 1, 0, 0, 0, 0, 0]

Then we can have cellStateNumber: [66, 8, 16, 1, 4, 32]

+ float64[6]      cellX #fixed, the x position of the buff cells

cellX: [0.5, 1.9, 4.1, 7.6, 6.2, 4.0]

+ float64[6]      cellY #fixed, the y position of the buff cells

cellY: [3.36, 1.935, 4.59, 1.74, 3.165, 0.51]

+ RobotPose[4]    robotPose # four robot pose


E.g.

robotPose: 
  
    x: 0.647561371326
    y: 3.38313245773
    yaw: 0.040313269943
   
    x: 0.532608807087
    y: 4.69925546646
    yaw: 0.0332103334367
  
    x: 7.73535680771
    y: 4.55239772797
    yaw: 3.14024424553
  
    x: 7.72630119324
    y: 0.563540756702
    yaw: 3.13108444214



+ float64         gameTime
 
Start from 0.0

### 3.Rules

Each game has 3 mins. The buff cells refresh the state at 0, 60 and 120 secs.

The debuff and buff cells refresh symmetrically.

