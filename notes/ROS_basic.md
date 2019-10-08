# ROS Basic

- [ROS Basic](#ros-basic)
  - [Starting ROS](#starting-ros)
  - [Move around](#move-around)
  - [Create a catkin workspace](#create-a-catkin-workspace)

## Starting ROS

1. Run roscore first:

    ```bash
    roscore
    ```

2. Open another terminal, and launch:

    ```bash
    roslaunch husky_gazebo husky_empty_world.launch
    ```

    ```bash
    roslaunch husky_gazebo husky_maze2.launch
    ```

## Move around

- To go to the installation location
  
    ```bash
    roscd
    ```

  - Or to the location of an specific package

  for example

  ```bash
  roscd roscpp
  ```
  
  ```bash
  roscd [pkg_name]
  ```

- To find a package

    ```bash
    rospack find [pkg_name]
    ```

## Create a catkin workspace

 > currently the **COMORO_project** is the catkin_workspace having a package for each homework

- Source it

    ```bash
    source devel/setup.bash
    ```

- To know the current source location. :+1: to have a correct it had to be included besides the **/opt/ros/melodic/share** directory

    ```bash
    echo $ROS_PACKAGE_PATH
    ```
