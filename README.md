<a name="readme-top"></a>

# tello_gesture_steering
ROS2 project for operating DJI Ryze Tello drone using hand gestures

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#building-from-source">Building from source</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#demonstration">Demonstration</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>
<br>

## About the project
This project was made as a dissertation for CS studies at the Univeristy of Wrocław. The project is easy to integrate with any drone that runs on ROS2. You can check full documentation of the project in the [thesis.pdf](./thesis.pdf) file. <br>
The project allows users to move a flying drone using bare hands, without any controllers.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting Started
### Prerequisites
* **TensorFlow Lite** - needed to run the neural network model for gesture recognition. Installation commands:
  ```
  sudo apt install python3-pip
  pip3 install tflite-runtime
  ```
* **ros galactic** - the base framework for running this project. Installation guide available on [ros_installation] *(The project was built and tested on ros galactic; working on different ros2 distro not guaranteed)* <br>
    - You will also need a ros workspace; if you don't have any, follow [ros_workspace] tutorial on creating one (clone this repo in step 3 of the tutorial).
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### ROS packages
The projects depends on two additional ROS2 packages:
* **[tello_ros]** - a ros2 package integrating DJI Ryze Tello drone with ROS2 environment; needed for other programs to communicate with the robot.
* **[ros2_shared]** - a ros2 package needed for compilation of ros nodes

Follow the links to github repositories and clone them into your workspace.
***Every ros package should be downloaded / cloned into `src` directory of your workspace***
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Building from source
*Most of the instructions here are the steps from [ros_workspace] tutorial, but with instructions specified for this project, so if you are not a newbie with ros, you can skip those steps.*<br><br>

#### Installing dependecies
Having all needed packages in `src` directory (including this one), you need to resolve dependencies. To do so, type in terminal:
```
rosdep init #(only if you are using rosdep for the first time ever)
rosdep update
rosdep install --from-paths src -iy
```
***You should swich `src` with the relative path to the `src` directory of your workspace.***
<p align="right">(<a href="#readme-top">back to top</a>)</p>

#### Building Workspace
Having the dependencies installed, you can build your workspace, to do so type:
```
colcon build --symlink-install
```
*You need to have some ros workspace sourced to perform this command.*

This command will create 3 directories, that you will need later for running the programs, so execute it inside your ros workspace directory.

If everything went without any errors, you should be ready to launch the programs.
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage
To run the program you need to be connected with the drone.<br>
Each program needs to be run in separate terminal (remember that everyone of them needs to have your ros workspace sourced - `source /<path_to_workspace>/install/setup.bash` - replace `.bash` with your shell). <br><br>
***If you want to integrate this project with your drone, check the `ROS API (3.3)` chapter in [thesis.pdf](./thesis.pdf) .***

### tello_bringup
Runs basic communication with the drone, and provides base topics and services, as well as video interface for the drone view.
```
ros2 launch tello_teleop base_setup.launch.xml
```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### tello_teleop
Operate the drone using a controller. Key mapping (for ps4 controller):
- **square** -  turbo button, makes the drone move faster
- **triangle** - emergancy button, power off the drone's motors
- **R1 + R2 + x** - takeoff
- **R1 + R2 + circle** - land
- **R1 + left stick** - movement in x (stick front / back - movement front / back) and y (stick left / right - movement left / right) axes 
- **R1 + right stick** - movement in z axis(stick up / down - movement up / down) and drone rotation (stick left / right - rotation left/right)
- **L1 + arrows** - flips (arrow up - front, arrow down - back, arrow left - left, arrow right - right)

```
ros2 launch tello_teleop joy_teleop.launch.xml
```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### gesture_steering
The main program, runs the gesture steering node. 
```
ros2 launch gesture_steering gesture_steering.launch.xml
```
Due to configurable interface, you can change actions performed by the drone on each gesture with the config files:
- there are two parameters in yaml files inside the `config` directory:
  - gestures: list of available gestures to detect (names of the gestures)
  - func_arguments: a parameter with sub parameters where the sub parameter name corresponds to the string form `gestures` parameter, and it's value are the argument passed to the function performing actions. Based by the arguments, the actions is different:
    - *list of 4 integers `(-1/0/1)`*: a movement action specyfing movement axis (`linear_x`, `linear_y`, `linear_z`, `angular_z`) and movement direction (`-1`- backward, `1` - forward)
    - *list of 1 integer `(any number)`*: landing
    - *list of 1 character `(f/b/l/r)`*: flip instructions, the character specifies the direction of the flip (`front`, `back`, `left`, `right`)
  
  If you are lost with this description try to look on [no_flips.yaml](./gesture_steering/config/no_flips.yaml) or [steering_params.yaml](./gesture_steering/config/steering_params.yaml) files, and analyze them acording to these instructions. <br>
  To see the hand gestures that are recognized by the system check chapter `gestures (6.2)` in [thesis.pdf](./thesis.pdf). 
  
***To each of those commands you can add `--show-args` flag, which will show available arguments with documentation***
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Demonstration
<!-- [![Watch the video](https://img.youtube.com/vi/_Ze-3QzduGk/maxresdefault.jpg)](https://youtu.be/_Ze-3QzduGk) -->
<video src="https://youtu.be/_Ze-3QzduGk" width=180/>
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Contact
Aleksander Szymański - bitterisland6@gmail.com
<p align="right">(<a href="#readme-top">back to top</a>)</p>

[ros_installation]: https://docs.ros.org/en/foxy/Installation.html
[ros_workspace]: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
[tello_ros]: https://github.com/clydemcqueen/tello_ros
[ros2_shared]: https://github.com/ptrmu/ros2_shared