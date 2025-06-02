# Localization

## Adaptive-Monte-Carlo-Localization

Needed Data:

- map -> to filter out wrong particles
- forward model -> to predict the next state of the particles according to the motion model
- laserscan -> to update the particles according to the measurement model
- commands and start position -> to initialize the particles

Read probabilistic robotics:

Idea: bei großer flacher gaußglocke und kleiner spitzer -> müssen nicht zusammenhängen -> kann man trennen

## Coordinate Systems

- Point representation: 2D point in the world coordinate system: `p = (x, y)` tells 3 things:
  - Where point is located
  - Direction vector from origin to point
  - how to get there (vector representation) $p = x * e_x + y * e_y$
- Rotation: via a rotation matrix:
  $$ D = \begin{pmatrix}
    cos(\theta) & -sin(\theta) \\
    sin(\theta) & cos(\theta)
  \end{pmatrix} $$
  - Rotation in 3d is a bit more complicated -> it is relevant what axis is rotated around first
  - 3x3 matrix ->

Frames / Transforms:

- 4x4 matrix
  - Top left -> 3x3 rotation matrix
  - right 3x1 -> translation vector position from origin
  - bottom row -> (0, 0, 0, 1) 1 is scalar -> 0s for schiefe koordinaten systeme
  - Describes position and orientation in 3D, Direction vector from origin to point, How to get there
- Allow for:
  - Relative position of Lidar on the robot
  - relative position from footprint of the robot to the center of the robot
  - relative position of the robot to the map
- Chain of transformations:
- T_Lidar point \* T_lidar_robot \* T_robot_map -> original point in map coordinates
  - &rarr; in ros -> TFTree -> are transforms between different coordinate systems

Problems with cos, sinus -> are not possible -> sometimes 0 -> cant devide by 0 so not invertable:

Idea: use quaternions -> 4D vector -> 3D rotation (so in 2D make a qube around the 2d vector):

- with the condition that the length of the quaternion (norm) is 1
- $q = (q_0, q_1, q_2, q_3)$

&rarr; used in ROS in amcl and internally in tftree

-> use ```getYawfromQuaternion``` to get the yaw angle from the quaternion and ```quaternionFromYaw``` to get the quaternion from the yaw angle

-------

Task:

1. Ontop of wallfollower: connect to amcl + let amzl tell me where I am
2. Program that prints x,y position on screen:

1: shell program start and running
2: shell start rqt
3: shell start rviz and set parameters ->

Create launch files for rqt, rviz and then map_server that serves the map -> use rviz an static_transform_publisher

Werte für amcl positionen müssen gleich sein wie start punkt bei World &rarr; sonst kidnapped robot kann amcl nicht lösen

min, max particles is adaptive of amcl

laser max beams = 30 -> sucht in 30dim raum ob er position schon kennt

2 launch dateis bauen -> eine mit params für rvis
1 mit rvis + amcl config

Amcl punktwolke angucken

neues Program schreiben, dass die Position ausgibt

- bei amcl subscriben
- Position geben lassen
- Dann orientierung -> Quaternion in yaw umwandeln

Zusatzaufgabe: als ropra ros2 simulator testen

danach: von Punkt a nach punkt b fahren Pose: (x,y, alpha) -> fahren + drehen in Richtung

- Brauchen winkel von richtungsvektor zu b von a aus &rarr; ist zielorientierung: atan2
- Punkt bis 0,5 Meter dicht anfahren
- plan nutzen um richtig zu orientieren
