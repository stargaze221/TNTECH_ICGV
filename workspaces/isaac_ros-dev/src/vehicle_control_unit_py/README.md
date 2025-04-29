Now, I need to fully understand isaac_sim_example.launch.py

isaac_sim_example.launch.py
- nvblox_examples_bringup
  - launch/navigation/nvblox_carter_navigation.launch.py
    - nav2_bringup',
        - 'launch/navigation_launch.py  <--- This file does not need to be updated.

- nvblox_examples_bringup
  - launch/perception/nvblox.launch.py
    - # Add the nvblox node.
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=parameters,
    )

- nvblox_examples_bringup
  - launch/visualization/visualization.launch.py

