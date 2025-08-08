 # Interface messages for AMR package

 The SetFlag and GroundTruth service messages are exposed in this package

 ```SetFlag```
 flag - Character from {a, o, d, g, w, s, l}
 a - Appends current robot location
 o - Overwrites robot location with current location and index i
 d - Deletes waypoint at index i
 g - Goes to waypoint described by index i and whether docking or not
 w - Deletes all waypoints and appends current location
 s - Saves The docking location with an aruco marker
 l - Saves last ground truth position of the robot as current robot position

 index - Number from 0 - (# of waypoints - 1)
 Describes an index that is needed for certain flags, such as o, d, and g

 docking - if 'd' is passed in, docking is enabled. Anything else doesnt do anything

 label - String, name for a waypoint

 ```Ground Truth```
 goal - The pose of the goal 
 last_ground_truth - The pose of the last ground truth know for the robot

 Both of these are needed for the recovery phase
