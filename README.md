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
- NVblox with Realsense Camera (docker)
```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py
```
- Interface with Arudino that sends and receives PWM signals.
- The navigation stack is running but the interface is not working as intended.

## The Current Task
- Reverse engineering with ISAAC SIM Example
    - I need to see the TF tree
    - Also, I need to see the data stream also.
    - I spent already a lot of time trying random explorative changes. I feel that I am repeating the same thing over and over. 
    - I must look at ISAAC SIM Example on my desktop computer.