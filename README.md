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
Snippet(launch base):
``` bash
ros2 launch ir_robots kobuki.launch.py # Driver of the kobuki
```
-----------------------------------------------------------------------

-----------------------------------------------------------------------
Snippet(launch avoid_obstacle_cibernots):
``` bash
ros2 launch avoid_obstacle_cibernots avoid_obstacle.launch.py  # avoid_obstacle_cibernots
```
-----------------------------------------------------------------------

## AvoidObstacleNode

## Finit State Machine

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
