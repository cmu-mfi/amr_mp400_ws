This package is used to set up a service-client communication that saves poses of the robot to a text file and can publish those positions to the goal_pose topic and navigate to it

### Steps to Run Service and Client:

Once your workspace is built and sourced:

1) From one terminal, run:

   ` ros2 run waypoint_maker waypoint_service `

   You should now see that the service is running in the terminal, as there should be no output or input from the terminal

2) Opening another terminal source the install file again and run:

   ` ros2 run waypoint_maker waypoint_client <f> <i> <d> <label>`

   where f, i, d <label> are the flags, index, docking, and label as described in amr_mp400_interfaces

