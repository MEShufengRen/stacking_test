# Tower Stacking
**Team Members: Jiarui Li, Qinjie Lin, Shufeng Ren, Patricia Sung, Yuchen Wang**

*Northwestern University ME 495: Embedded Systems in Robotics (Fall 2018)*


## Introduction
#### Objective
The goal of this project is for Baxter to pick different blocks on the table and stack them together.

[![Overview](https://img.youtube.com/vi/SKFFj9aCwcA/0.jpg)](https://www.youtube.com/watch?v=SKFFj9aCwcA)

#### Basic Introduction
The Baxter is able to recognize all the blocks on the table through image processing. The location, size and color information of the blocks will be passed to the robot and it will calculate the goal position of each movement. The robot arm picks up the largest block first and places it to a fixed position. Then it repeats the previous step by finding a smaller block and stack it on the previous one.

All the blocks were 3D printed, with different size ranging from 2"x2"x2" up to 3"x3"x2". The color of the blocks are either red or yellow.

#### Computer Vision
Baxter's right hand camera has a decent resolution that works well in the project, so there is no need of using an external web cam. A node was written to process the raw image data basically using OpenCV library. When the user clicks on the camera image view and picks a color, the node would fetch the RGB value of the pixel being clicked and set a boundary range based on that value (in our case the boundary was **([B-30, G-30, R-30],[B+30, G+30, R+30])**). All the contours that matches the RGB boundary would be detected and labeled. Then the contours will be sorted by area and listed from the largest to the smallest. The node will finally publish a topic containing the center coordinates of all the contours(sorted by area) in pixel value, and the angles that each contour tilted with respect to x-axis.

### Translation from Pixel Coordinate to World Coordinate
* Our group is using the baxter right hand camera to capture the blocks' position on the table. Their positions are caculated from the blocks' pixel coordinates published by the computer vision. The caculation is based on the following function.  
<a href="https://www.codecogs.com/eqnedit.php?latex=s\begin{bmatrix}u\\v\\1\end{bmatrix}=\begin{bmatrix}\alpha_x&space;&&space;0&space;&&space;u_0&space;&&space;0\\&space;0&space;&&space;\alpha_y&space;&&space;v_0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0\end{bmatrix}\begin{bmatrix}R&space;&&space;t\\0&space;&&space;0\end{bmatrix}\begin{bmatrix}x_\omega\\&space;y_\omega\\&space;z_\omega\\&space;1\end{bmatrix}=M_1M_2X_\omega" target="_blank"><img src="https://latex.codecogs.com/gif.latex?s\begin{bmatrix}u\\v\\1\end{bmatrix}=\begin{bmatrix}\alpha_x&space;&&space;0&space;&&space;u_0&space;&&space;0\\&space;0&space;&&space;\alpha_y&space;&&space;v_0&space;&&space;0\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0\end{bmatrix}\begin{bmatrix}R&space;&&space;t\\0&space;&&space;0\end{bmatrix}\begin{bmatrix}x_\omega\\&space;y_\omega\\&space;z_\omega\\&space;1\end{bmatrix}=M_1M_2X_\omega" title="s\begin{bmatrix}u\\v\\1\end{bmatrix}=\begin{bmatrix}\alpha_x & 0 & u_0 & 0\\ 0 & \alpha_y & v_0 & 0\\ 0 & 0 & 1 & 0\end{bmatrix}\begin{bmatrix}R & t\\0 & 0\end{bmatrix}\begin{bmatrix}x_\omega\\ y_\omega\\ z_\omega\\ 1\end{bmatrix}=M_1M_2X_\omega" /></a>
* The intrinsic matrix <a href="https://www.codecogs.com/eqnedit.php?latex=M_1" target="_blank"><img src="https://latex.codecogs.com/gif.latex?M_1" title="M_1" /></a> is get from the camera calibration using ROS package **camera_calibration** . The external matrix <a href="https://www.codecogs.com/eqnedit.php?latex=M_2" target="_blank"><img src="https://latex.codecogs.com/gif.latex?M_2" title="M_2" /></a> is obtained from the TF tree from camera to base. Since the table is flat, the z coordinate of the blocks can be obtained from the IR range sensor in the left hand.

#### Robot Control
Baxter was programmed to begin at the same initial position each time. Then when list of blocks' positions reached, the robot arm were programmed to reach the certain height over the block, and then the robot arm came down to pick the block. After that, the arm moved to the goal position to place the blocks.

For the robot arm control, Cartesian Path Planner Plug-In in MoveIt was used to generate waypoints between beginning position and goal position. So that the robot arm won't move in unusual way and avoid hitting obstacles. And then, MoveIT planner was used to generate plan to make the arm move to the goal position along the waypoints.

During the movement, the sensor in the left hand was used to detect whether the block was attached to the baxter's left hand successfully or not. If the distance from left hand was less than 0.1m, the baxter will assume that the block was not picked successfully. Then in the next placement, the height of the block being placed will not increase and also the baxter would came to the pick block again, which were not being picked successfully at last time.


## Implementation
#### Launch
[`demo_baxter.launch`](launch/demo_baxter.launch)  
[`move_group.launch`](launch/move_group.launch)  
[`stacking_part1.launch`](launch/stacking_part1.launch)  
[`stacking_part2.launch`](launch/stacking_part2.launch)  

#### Nodes
##### Computer Vision Node
[`image_process.py`](src/image_process.py)

Subscribed Topic:  
`/cameras/right_hand_camera/camera_info`  
`/cameras/right_hand_camera/image`  
`/Confirm_pos`  

Published Topic: `/location`


##### Camera Calibration Node
[`RM_Qua.py`](src/RM_Qua.py)

Subscribed Topic:  
`/location`  
`/robot/range/left_hand_range/state`

Published Topic: `/world_location`


##### Robot Control Node
[`position_sub_client.py`](src/position_sub_client.py)

Subscribed Topics: `/world_location`

Client: `move_arm`

[`IKServer.py`](src/IKServer.py)

Subscribed Topics: `/robot/range/left_hand_range/state`

Server: `move_arm`  

[`left_moveit.py`](src/left_moveit.py)


##### Starting Task Node
[`Confirm_Pos.py`](src/Confirm_Pos)

Published Topic: `/Confirm_pos`

## Work Process
1. Nodes Starting(Debugging Mode)
```bash
rosrun baxter_interface joint_trajectory_action_server.py
```
```bash
roslaunch stacking_test demo_baxter.launch
```
```bash
rosrun stacking_test IKServer.py
```
```bash
rosrun stacking_test image_process.py
```
```bash
rosrun stacking_test RM_Qua.py
```
```bash
rosrun stacking_test position_sub_client.py
```
Then click blocks in the camera image window, make sure only blocks are selected without noise points.
```bash
rosrun stacking_test Confirm_Pos.py
```
Once completed and started another task, close the Confirm_Pos node and start it again.  
2. Launch Starting(Working Mode)
```bash
rosrun baxter_interface joint_trajectory_action_server.py
```
```bash
roslaunch stacking_test stacking_part1.launch
```
Then click blocks in the camera image window, make sure only blocks are selected without noise points.
```bash
roslaunch stacking_test stacking_part2.launch
```
Once completed and started another task, close the stacking_part2.launch and start it again.

## Conclusion

#### Further Improvements
The team could improve the project by making a **Hanoi Tower Game**.
We could also further improve our controller so that Baxter could accomplish obstacle avoidance during trajectory movement.