# tello_gesture_steering
Gesture steering project for DJI Ryze Tello drone using ROS2

## Installation
### Needed programs
* **TensorFlow Lite** - needed to run the neural network model for gesture recognition. Installation commands:
  ```
  sudo apt install python3-pip
  pip3 install tflite-runtime
  ```
* **ros galactic** - the base framework for running this project. Installation guide available on [ros_installation], but this command should work:
  ```
  sudo apt install ros-galactic-desktop
  ```
  You can change *galactic* with other ros2 distribution, but this version is recommended, as it is the version that the project was tested on.

### ROS packages
To install the following ros packages, you need to have a ROS workspace. To create one (if you don't have) follow creating [ros_workspace] tutorial. <br>
***Every ros package should be downloaded / cloned into `src` directory of your workspace***
* **[tello_ros]** - a ros2 package integrating DJI Ryze Tello drone with ROS2 environment; needed for other programs to communicate with the robot.
* **[ros2_shared]** - a ros2 package needed for compilation of ros nodes

Follow the links to github repositories and clone them into your workspace.
### Building the workspace
#### Installing dependecies
Having all needed programs installed, you need to clone this repository (into `src` directory of the ros workspace). Once every file is downloaded you need to download the dependencies specified in `package.xml` files. To do so type (beeing in your workspace directory):
```
rosdep update
rosdep install --from-paths src -iy
```
***It's best to execute those commands beeing in the workspace directory, but you can also do this from anywhere providing path to the `src` directory of your workspace.***
#### Building
Having the dependencies installed, you can build your workspace, to do so type:
```
colcon build --symlink-install
```
This command will create 3 directories, that you will need later for running the programs, so execute it inside your ros workspace directory.

If everything went without any errors, you should be ready to launch the programs.

## Running the programs
To run the program you obviously need a drone. Once it's on, you need to connect to its access point. When connected to the drone's network, you are ready to run the programs.<br>
Each program needs to be run in separate terminal (remember that everyone of them needs to have your ros workspace sourced - `source /<path_to_workspace>/install/setup.bash` - replace `.bash` with your shell).

### tello_bringup
Runs basic communication with the drone, and provides base topics and services, as well as video interface for the drone view.
```
ros2 launch tello_teleop base_setup.launch.xml
```

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

### gesture_steering
The main program, runs the gesture steering node
```
ros2 launch gesture_steering gesture_steering.launch.xml
```

***To each of those commands you can add `--show-args` flag, which will show available arguments with documentation***




[ros_installation]: https://docs.ros.org/en/foxy/Installation.html
[ros_workspace]: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
[tello_ros]: https://github.com/clydemcqueen/tello_ros
[ros2_shared]: https://github.com/ptrmu/ros2_shared