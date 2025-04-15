# TNTECH_ICGV

## Building Modified Isaac Ros Environment

After cloning this repo, download the sources from the other referenced repos.
```bash
yoonlab07nano@ubuntu:~$ git submodule update --init --recursive
```

It is convenient to set the `ISAAC_ROS_WS` environment variable to your `.bashrc` file:
```bash
yoonlab07nano@ubuntu:~$ echo 'export ISAAC_ROS_WS=/home/<username>/TNTECH_ICGV/workspaces/isaac_ros-dev' >> ~/.bashrc
yoonlab07nano@ubuntu:~$ source ~/.bashrc
```

Also, to append my docker file, e.g., `isaac_ros_common/docker/Dockerfile.yoonlab`, i.e., 
add the following line to the config file located at `isaac_ros_common/scripts/.isaac_ros_common-config`:
```bash
yoonlab07nano@ubuntu:~$ echo 'CONFIG_IMAGE_KEY=ros2_humble.realsense.yoonlab' >> isaac_ros_common/scripts/.isaac_ros_common-config
```
If the file does  not exist then add the file in the path.

```bash
yoonlab07nano@ubuntu:~$ cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

After the Docker images are built, you are in the container. Build the isaac_ros packages.

```bash
admin@ubuntu:/workspaces/isaac_ros-dev$ colcon build
```


## The Current Status
- NVblox with Realsense Camera
    - 