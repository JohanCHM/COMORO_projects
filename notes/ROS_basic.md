# ROS Basic

- [ROS Basic](#ros-basic)
  - [Starting ROS](#starting-ros)
  - [Move around](#move-around)
  - [Create a catkin workspace](#create-a-catkin-workspace)
    - [Structure of a catkin package](#structure-of-a-catkin-package)
    - [How-to](#how-to)
      - [To create a new catkin project](#to-create-a-new-catkin-project)

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

### Structure of a catkin package

```txt
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

### How-to

- Source it

    ```bash
    source devel/setup.bash
    ```

- To know the current source location. :+1: to have a correct it had to be included besides the **/opt/ros/melodic/share** directory

    ```bash
    echo $ROS_PACKAGE_PATH
    ```

#### To create a new catkin project

  1. Move into the **src** of the **workspace_folder**

      ```bash
      cd ~/COMORO_projects/src
      ```

  1. Create the package

      ```bash
      catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
      ```

      for example

      ```bash
      catkin_create_pkg examplePkg std_msgs rospy roscpp
      ```

  1. Source the file

      ```bash
      cd ~/COMORO_projects
      catkin_make
      ```
