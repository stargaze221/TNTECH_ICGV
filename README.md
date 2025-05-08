# TNTECH_ICGV

## Building Modified Isaac Ros Environment

After cloning this repo, download the sources from the other referenced repos.(host)
```bash
git submodule update --init --recursive
cd workspaces/isaac_ros-dev/src/isaac_ros_common
git checkout tntech_igvc
```

It is convenient to set the `ISAAC_ROS_WS` environment variable to your `.bashrc` file:(host)
```bash
echo 'export ISAAC_ROS_WS=/home/<username>/TNTECH_ICGV/workspaces/isaac_ros-dev' >> ~/.bashrc
source ~/.bashrc
```

Also, to append my docker file, e.g., `isaac_ros_common/docker/Dockerfile.yoonlab`, i.e., 
add the following line to the config file located at `isaac_ros_common/scripts/.isaac_ros_common-config`:(host)
```bash
echo 'CONFIG_IMAGE_KEY=ros2_humble.realsense.yoonlab' >> isaac_ros_common/scripts/.isaac_ros_common-config
```
If the file does  not exist then add the file in the path.(host)
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

After the Docker images are built, you are in the container. Build the isaac_ros packages.(docker)

```bash
colcon build
```

To rean Realsense example (docker)
```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-up-to realsense_splitter
source install/setup.bash
```

## The Current Status
- Navigation Stack Works (docker)
```bash
ros2 launch vehicle_control_unit_py navigation_launch.py 
```
## The Current Task
- Now, I need to set up controller using the following nodes that I drafted.
    - cmdvel_to_pwm_with_rc_node.py
    - sketch_apr28.ino

- I need to add some development that uses:
    - Visual Odom to see whether it follows the command
    - The Ackerman ...

- Immediate work to do:
    - Use the velocity_ctrl_dev.launch.py
    - Log odom, cmd, pwm