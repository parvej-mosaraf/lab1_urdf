# lab1_urdf

Repository: https://github.com/parvej-mosaraf/lab1_urdf.git

This package contains a simple URDF robot description, a launch file to display the robot in RViz, and an RViz configuration for visualization. It's intended for learning and demonstration of URDF, robot_state_publisher, and RViz in ROS 2.

## Contents

- `CMakeLists.txt` - package CMake and install rules.
- `package.xml` - package manifest.
- `launch/display.launch.py` - launch file to start the robot description and RViz display.
- `urdf/lab1_robot.urdf` - the URDF model of the robot.
- `rviz/lab1_view.rviz` - an RViz configuration to view the robot.

## Quick contract

- Inputs: a working ROS 2 environment and this package installed in a ROS 2 workspace.
- Outputs: robot model visualized in RViz; topics published by robot_state_publisher.
- Success criteria: RViz opens and the robot described in `urdf/lab1_robot.urdf` is visible using the provided launch file.

## Prerequisites

- ROS 2 (a supported distribution installed on your system). Examples: Humble, Iron, or later.
- colcon (for building the workspace).
- rviz2 and robot_state_publisher (commonly available as part of ROS 2 desktop/install bundles).

Note: Replace `<ros2-distro>` with your distribution name where relevant (e.g., `humble`).

## Build (WSL / Ubuntu)

Open a terminal in the workspace root (the parent of this package). Example steps (bash):

```bash
# source your ROS 2 installation (replace <ros2-distro> if needed)
source /opt/ros/<ros2-distro>/setup.bash

# from your ROS 2 workspace root (for example ~/ros2_ws)
colcon build --packages-select lab1_urdf

# source the install overlay
source install/setup.bash
```

If you use PowerShell on Windows and have ROS 2 installed there, adapt the commands for PowerShell and your ROS setup accordingly.

## Run / Visualize

Use the provided launch file which starts the robot_state_publisher (or otherwise publishes the URDF) and opens RViz with the included configuration.

```bash
ros2 launch lab1_urdf display.launch.py
```

After the launch completes, RViz should open (or you can open RViz and load `rviz/lab1_view.rviz`) and the robot from `urdf/lab1_robot.urdf` should appear under the RobotModel display.

If the robot does not appear:
- Make sure the `robot_state_publisher` or equivalent node is running and publishing TF.
- Check `ros2 topic list` to ensure relevant topics are present (e.g., `/tf`, `/robot_description` or joint states topics depending on the launch file).

## Files of interest

- `urdf/lab1_robot.urdf`: The robot description. Edit or extend links/joints and materials here.
- `launch/display.launch.py`: Launch logic for publishing the URDF and starting RViz. You can extend it to add controllers or spawn simulated joints.
- `rviz/lab1_view.rviz`: Saved RViz layout used by the launch file.

## Troubleshooting

- If RViz complains about missing TF frames, ensure the node publishing TF is active and the frame names in URDF match the TF frames expected in RViz.
- If you see only an empty grid, try resetting the fixed frame in RViz to the robot base frame (commonly `base_link` or similar).

## Contributing

1. Fork the repository.
2. Create a branch for your change.
3. Open a pull request with a clear description of the change.

## License

This repository currently does not specify a license in `package.xml`. Add a LICENSE file and update `package.xml` if you intend to publish or share it with a specific license.

## Contact

Maintainer: parvej (see `package.xml`).

---

If you want, I can also:
- add a short README section explaining how to edit the URDF (materials, inertial tags, joint limits),
- or add a small example script that publishes sample joint states so you can observe joint motion in RViz.
