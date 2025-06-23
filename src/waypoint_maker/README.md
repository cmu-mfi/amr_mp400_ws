This package is used to set up a service-client communication that saves poses of the robot to a text file and can publish those positions to the goal_pose topic and navigate to it

### Steps to Run Service and Client:

Once your workspace is built and sourced:

1) From one terminal, run:

   ` ros2 run waypoint_maker waypoint_service `

   You should now see that the service is running in the terminal, as there should be no output or input from the terminal

2) Opening another terminal source the install file again and run:

   ` ros2 run waypoint_maker waypoint_client <f> <i> <d>`

   where f, i, d are the flags, index, and docking respectively

### Flags and Indices: 
**a** -> Append a pose to the file; Index doesnt matter for this operation but is still needed to be input

**o** -> Overwrite a pose at index i with the current robot pose

**d** -> Delete a pose at index i

**w** -> Delete all poses and add the current robot pose as the only pose in the file

**g** -> Publish pose described at index i to the goal_pose topic

### Docking Toggle:

**d** -> Starts docking procedure after navigation completes

**Note:** Anything in this option will not do anything except for the letter d, but is still needed for program to run
