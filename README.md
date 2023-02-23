# Avoid_obstacle_cibernots

<div align="center">
<img width=400px src="?raw=true" alt="explode"></a>
</div>

<h3 align="center">Avoid Obstacle </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [How to execute the programs](#How-to-execute-the-programs)
- [AvoidObstacleNode](#AvoidObstacleNode)
- [Finit State Machine](#Finit-State-Machine)
- [Lidar logic](#Lidar-logic)
- [Parameters](#Parameters)
- [Implements](#implements)
- [Tests](#tests)
- [Continuous integration](#Continuous-integration)
- [license](#license)


## How to execute the programs

<div align="center">
<img width=600px src="?raw=true" alt="explode"></a>
</div>

First connect the base and the lidar then :
-----------------------------------------------------------------------
Snippet (launch base):
``` bash
ros2 launch ir_robots kobuki.launch.py # Driver of the kobuki
```
-----------------------------------------------------------------------

-----------------------------------------------------------------------
Snippet (launch avoid_obstacle_cibernots):
``` bash
ros2 launch avoid_obstacle_cibernots avoid_obstacle.launch.py  # avoid_obstacle_cibernots
```
-----------------------------------------------------------------------

## AvoidObstacleNode

## Finit State Machine
The finit state machine consists of 6 states: Forward, Turn, Stop, Reor and Arch.

*Forward*: straight ahead state. If no measurements are received from the laser, it jumps to the *Stop* state. On the other hand, if an obstacle is detected by the use of the laser, it jumps to the *Turn* state.

*Turn*: pi/2 radian turning state. The direction of the pi/2 radians turn will be carried out according to the side on which the obstacle is detected, turning in the opposite direction to the obstacle. Once the turn has been made, it jumps to the *Arch* state.

*Arch*: state of progress in arc. If the robot detects any obstacle in the process of advancing in arc, it will jump to the *Turn* state. Once the arc path is completed, it jumps to the *Reor* state.

*Reor*: Reorientation state. At the end of the arc, the robot reorients itself back to the direction it had just before encountering an obstacle in the *Forward* state.

*Stop*: Stop state. If no readings are received from the laser, the robot stops until readings are received again, in which case it will jump to the *Forward* state.

A video of the operation of the robot with the finite state machine implemented is attached below:
[Video](https://urjc-my.sharepoint.com/:v:/g/personal/da_quinga_2020_alumnos_urjc_es/EbifwcoNsNhFtt5DDTsoIvEBRNQRaVDX59bKmEr3KTWU8g?e=4hNRhZ)

## Lidar logic

In this case, we used the /scan_filtered topic, which published an array with a length of 360. This array consists of the distance values taken by the lidar. 
The logic is: the lidar starts its array (0) at position π/2. For this reason, the values of importance, or risk, are those ranging from π/4 to 3π/4, which these measurements correspond to the objects in front of the kobuki, they can be related to the values 0-45 and 315-360 of the array. 
In this case, it is very simple and visual because the array has exactly 360 measures, in the case that the number of measures would be different, we could make this calculation:

Snippet (example):
``` c++
val_π/4 = 45*x/360; //example for 45º.
```
In this way, we could get the corresponding number from the array, relating it to an angle.

With all this in mind, we simply create two loops, one for each piece of the array, corresponding one zone to the right of the robot (π/2-π/4) and the other to the left (π/2-3π/4). In each iteration of the loop we put a series of conditions to filter the measurements, in case we detect something in our distance threshold, the robot will store that distance as well as notifying the detection by a boolean.

After setting our boolean to true, we must indicate on which side we have detected the obstacle. In this step, we were very careful because if the robot entered a corner and this was not very well determined, the robot would get stuck.

We went through our previously saved array of measurements and kept the position of the smallest measurement. Depending on the position, we could know whether the nearest object is to the left or to the right. For example, if we take 80 measurements and start by measuring the right side of the robot, if our smallest measurement is between position 0-39, it means we have an obstacle on the right.

## Parameters
We used .yaml and .py files:

### .yaml



-----------------------------------------------------------------------
Snippet(yaml example):
``` yaml

```
-----------------------------------------------------------------------

### .py



-----------------------------------------------------------------------
Snippet(launch example):
``` py

```
-----------------------------------------------------------------------

### How to get the params


-----------------------------------------------------------------------
Snippet(Getting params example):
``` cpp

```
-----------------------------------------------------------------------

## Implements


### Led Implement

-----------------------------------------------------------------------
Snippet(Led Implement):
``` cpp

```
-----------------------------------------------------------------------
### Sound Implement



-----------------------------------------------------------------------
### Bumper Implement
Snippet(bumper Implement):
``` cpp

```
-----------------------------------------------------------------------


## Tests


## Continuous integration


## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
