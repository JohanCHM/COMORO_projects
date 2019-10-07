# How to open a world

1. Position your `*.world` in a known location. One posibility is to work with the husky file.

    ```bash
    cd /opt/ros/melodic/share/husky_gazebo/worlds
    ```

1. Create a launch file. The same can be done relative to husky.

    ```bash
    cd /opt/ros/melodic/share/husky_gazebo/launch
    ```

    Inside that create a `*.launch`

    with the structure (in the following structure replace the maze2.world with your world.)

    ```xml
    <launch>

        <arg name="laser_enabled" default="true"/>
        <arg name="kinect_enabled" default="false"/>

        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="world_name" value="$(find husky_gazebo)/worlds/maze2.world"/>
            <arg name="paused" value="false"/>
            <arg name="use_sim_time" value="true"/>
            <arg name="gui" value="true"/>
            <arg name="headless" value="false"/>
            <arg name="debug" value="false"/>
        </include>

        <include file="$(find husky_gazebo)/launch/spawn_husky.launch">
            <arg name="laser_enabled" value="$(arg laser_enabled)"/>
            <arg name="kinect_enabled" value="$(arg kinect_enabled)"/>
        </include>

    </launch>
    ```

