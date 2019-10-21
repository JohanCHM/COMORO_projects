# Homework

## Prepare drone

1. Firmwared

    ```bash
    sudo systemctl start firmwared.service
    ```

1. Check the firmwared is alive

    ```bash
    fdc ping
    ```

    should return PONG

1. Check the network interface

    ```bash
    ifconfig
    ```

    make sure it is your ethernet one

1. Launch the drone package

    > Check that you are connected to Internet if not it will fail

    ```bash
    sphinx /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
    ```

## Launch the drone

You have to source the bebop files

``` bash
source ~/bebop_ws/devel/setup.bash
```
 
### Launch the driver

``` bash
roslaunch bebop_driver bebop_node.launch
```

## Takeoff

```bash
rostopic pub --once bebop/takeoff std_msgs/Empty
```

## Land

```bash
rostopic pub --once bebop/land std_msgs/Empty
```
