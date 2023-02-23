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
[Video](https://urjc-my.sharepoint.com/:v:/g/personal/da_quinga_2020_alumnos_urjc_es/EVzN7Xe43xxArq__mGFdy_4B9yuPPSlcPpkyF0HMEDuCLw?e=weMXAx)

## Lidar logic

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
