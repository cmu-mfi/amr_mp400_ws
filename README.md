# amr_mp400_ws
ROS2 workspace for onboard packages

This package relies on the [neobotix packages](https://neobotix-docs.de/ros/ros2/installation.html) as well as the [docking_with_fiducial](https://github.com/cmu-mfi/amr_docking_fiducial/tree/main) package. 

To use this repository:

1) Clone and enter the repository into your working directory:

   ```
   git clone https://github.com/cmu-mfi/amr_mp400_ws.git
   cd amr_mp400_ws
   ```
   
2) Build the workspace and source install:

   ```
   colcon build --symlink-install
   source install/setup.bash
   ```
