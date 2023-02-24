# Avoid_obstacle_cibernots

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
- [Parameters](#parameters)
- [Implements](#implements)
- [Tests](#tests)
- [Continuous integration](#Continuous-integration)
- [license](#license)


## How to execute the programs

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
The finit state machine consists of 5 states: Forward, Turn, Stop, Reor and Arch.

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
val_45 = 45*x/360; //example for 45º (π/4)
```
In this way, we could get the corresponding number from the array, relating it to an angle.

With all this in mind, we simply create two loops, one for each piece of the array, corresponding one zone to the right of the robot (π/2-π/4) and the other to the left (π/2-3π/4). In each iteration of the loop we put a series of conditions to filter the measurements, in case we detect something in our distance threshold, the robot will store that distance as well as notifying the detection by a boolean.

After setting our boolean to true, we must indicate on which side we have detected the obstacle. In this step, we were very careful because if the robot entered a corner and this was not very well determined, the robot would get stuck.

We went through our previously saved array of measurements and kept the position of the smallest measurement. Depending on the position, we could know whether the nearest object is to the left or to the right. For example, if we take 80 measurements and start by measuring the right side of the robot, if our smallest measurement is between position 0-39, it means we have an obstacle on the right.

## Parameters
Use of parameters, implementation.

### .yaml

To make an improvement in our code, we used a .yaml file, very usefull when we have a considerable number of parameters, so we can control them in a unique file, and prevent a waste of time compiling our programs each time we vary a parameter. 

We can set up our program parameters in one unique file with this structure:

-----------------------------------------------------------------------
Snippet (.yaml example):
``` .yaml
# params.yaml
avoid_obstacle:
  ros__parameters:
    SPEED_LINEAR: 0.3
    SPEED_ANGULAR: 0.6
    OBSTACLE_DISTANCE: 0.5
    MULTIP_ARCH: 3.2
    MULTIP_TURN: 1.5
  
    use_sim_time: False
```
-----------------------------------------------------------------------

### .py

We created a new directory called "param", with its respective file "params.yaml". 
In the launcher, we need to get the path to the param file and create an os.path module that gives as acces to common pathname manipulations. 
```py 
# avoid_obstacle.launch.py
param_file = os.path.join(pkg_dir, 'param', 'params.yaml')
```
In the launcher, we need to configurate the node with the relation
```py 
# avoid_obstacle.launch.py
avoidobs_cmd = Node(
                    parameters =[param_file]
                    )
```

-----------------------------------------------------------------------
Snippet (launch example):
``` py
## avoid_obstacle.launch.py

# Get the path to the param file
pkg_dir = get_package_share_directory('avoid_obstacle_cibernots')
param_file = os.path.join(pkg_dir, 'param', 'params.yaml')

# Node configuration
avoidobs_cmd = Node(package='avoid_obstacle_cibernots',
                    executable='avoid_obs',
                    output='screen',
                    parameters=[param_file],
                    remappings=[
                        ('input_scan', '/scan_filtered'),
                        ('output_vel', '/cmd_vel'),
                        ('input_button', '/events/button'),
                        ('input_bumper', '/events/bumper'),
                        ('output_sound', '/commands/sound'),
                        ('output_led', '/commands/led1')
                    ])
```
-----------------------------------------------------------------------

### .cpp

Later, in our node, we need to declare parameters default values with
```cpp
// AvoidObstacleNode.cpp
declare_parameter("SPEED_LINEAR", 0.0f);
declare_parameter("SPEED_ANGULAR", 0.0f);
declare_parameter("OBSTACLE_DISTANCE", 0.0f);
declare_parameter("MULTIP_ARCH", 0.0f);
declare_parameter("MULTIP_TURN", 0.0f);
```
To get the parameters from the .yaml file we use
```cpp
// AvoidObstacleNode.cpp
get_parameter("SPEED_LINEAR", SPEED_LINEAR);
get_parameter("SPEED_ANGULAR", SPEED_ANGULAR);
get_parameter("OBSTACLE_DISTANCE", OBSTACLE_DISTANCE);
get_parameter("MULTIP_ARCH", MULTIP_ARCH);
get_parameter("MULTIP_TURN", MULTIP_TURN);
```


### How to get the params


-----------------------------------------------------------------------
Snippet(Getting params):
``` cpp
get_parameter("<param_name_yaml>", <parameter>);
```
-----------------------------------------------------------------------
### CMake

Finally, we need to generate the installation rules in the CMakeLists.txt file with
```CMake
# CMakeLists.txt
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
```

## Implements


### Led Implement
To see that the robot is in the "arc" state, a red LED lights up and in the "stop" state it lights up orange. In any other case it remains off.

-----------------------------------------------------------------------
Snippet(Led Implement):
``` cpp
// constructor
led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 10);

// case arc
out_led.value = kobuki_ros_interfaces::msg::Led::RED;

// case stop
out_led.value = kobuki_ros_interfaces::msg::Led::ORANGE;

led_pub_->publish(out_led);
```
-----------------------------------------------------------------------
### Sound Implement
In case of an emergency stop, i.e. the bumper is pressed, the robot stops and emits a sound.
snippet(sound implement):
```cpp
// constructor
sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);

// in bumperCallback
kobuki_ros_interfaces::msg::Sound out_sound;
out_sound.value = kobuki_ros_interfaces::msg::Sound::ERROR;
sound_pub_->publish(out_sound);
```

-----------------------------------------------------------------------
### Bumper Implement
If the bumper is pressed, it is considered that the laser has not detected any obstacle but there is one and therefore the boolean "pressed_" that controls whether or not to enter the state machine is set to false, which would stop the robot.

Snippet(bumper Implement):
``` cpp
// constructor
bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "input_bumper", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::bumper_callback, this, _1));

// Callback bumper
void
AvoidObstacle::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  pressed_ = false;
  .
  .
  .
}
```

-----------------------------------------------------------------------
### Button Implement
Si se pulsa un botón, "pressed_" que controla si se entra en la máquina de estados o no se pone a false o a true dependiendo del estado anterior. Siempre se pone al contrario del estado anterior.
Al empezar la ejecución "pressed_ = false" y hasta que no se pulsa el boton no comienza a moverse el robot. Al arrancar, basta con volver a pulsar el botón para parar el robot y este seguiría con el último estado registrado.
```cpp
// constructor
button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
  "input_button", rclcpp::SensorDataQoS(),
  std::bind(&AvoidObstacle::button_callback, this, _1));

// Callback button
void
AvoidObstacle::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED) {
    RCLCPP_INFO(get_logger(), "BUTTON PRESSED");
    pressed_ = !pressed_;
  }
}
```


## license 
<a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0"><img alt="Apache License" style="border-width:0" src="https://www.apache.org/img/asf-estd-1999-logo.jpg" /></a><br/>(Cibernots) </a><br/>This work is licensed under a <a rel="license" href="https://www.apache.org/licenses/LICENSE-2.0">Apache license 2.0
