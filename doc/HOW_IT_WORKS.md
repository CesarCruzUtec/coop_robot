# HOW IT WORKS

Automated exploration from the `multi_robot_exploration` package is a process that allows multiple robots to explore an unknown environment.

## LAUNCH FILES

### two_tb_exploration.launch

- Initialize the following parameters:
    - `model`: The model of the robots, in this case they are turtlebots `waffle_pi`.
    - `first_tb3`and `second_tb3`: The name of the two turtlebots.
    - `known_initial_pos`: If the initial position of the robots is known.
    - `first_tb3_x_pos`, `first_tb3_y_pos`: The initial position of the first turtlebot.
    - `second_tb3_x_pos`, `second_tb3_y_pos`: The initial position of the second turtlebot.
- Run the node [`map_node`](#map_node).
- Run the launch file [`spawn_robots.launch`](#spawn_robotslaunch) to spawn the two turtlebots.
    - `first_tb3_x_pos`, `first_tb3_y_pos`: The initial position of the first turtlebot.
    - `second_tb3_x_pos`, `second_tb3_y_pos`: The initial position of the second turtlebot.
- Run the launch file [`slam_online_synch.launch`](#slam_online_synchlaunch) to start the SLAM process for the two turtlebots.
    - `ns`: The namespace of each turtlebot.
- Run the launch file [`multi_robot_map_merge.launch`](#multi_robot_map_mergelaunch) to merge the maps of the two turtlebots.
    - `known_initial_pos`: If the initial position of the robots is known.
- Run the node `rviz` to visualize the map.
- Run the launch file [`modified_move_base.launch`](#modified_move_baselaunch) to start the navigation process for the two turtlebots.
    - `ns`: The namespace of each turtlebot.
- Run the node [`tb3_0_FE`](#tb3_0_fe) and [`tb3_1_FE`](#tb3_1_fe) to start the frontier exploration process for the two turtlebots.
- Run some `static_transform_publisher` nodes to publish the transformation between the two turtlebots and the map frame.
    - From `/tb3_ns/base_footprint` to `/tb3_ns/tb3_ns/base_footprint`.
    - From `/tb3_ns/tb3_ns/base_scan` to `/tb3_ns/base_scan`.

### spawn_robots.launch

- Initialize the following parameters:
    - `first_tb3_x_pos`, `first_tb3_y_pos`: The initial position of the first turtlebot.
    - `second_tb3_x_pos`, `second_tb3_y_pos`: The initial position of the second turtlebot.
    - `init_pose_0`, `init_pose_1`: The combined initial position for the two turtlebots.
- Run the launch file `empty_world.launch` to start the Gazebo simulation.
    - `world_name`: The name of the world file.
    - `paused`, `use_sim_time`, `gui`, `headless`, `debug`: The Gazebo simulation parameters.
- Run the launch file [`tb3_0.launch`](#tb3_*launch) to spawn the first turtlebot.
    - `init_pose`: The initial position of the first turtlebot.
    - `robot_name`: The name of the first turtlebot.
- Run the launch file [`tb3_1.launch`](#tb3_*launch) to spawn the second turtlebot.
    - `init_pose`: The initial position of the second turtlebot.
    - `robot_name`: The name of the second turtlebot.

### slam_online_synch.launch

- Initialize the following parameters:
    - `ns`: The namespace of each turtlebot.
- Run the node [`sync_slam_toolbox_node`](#sync_slam_toolbox_node) to synchronize the SLAM process for the two turtlebots.
    - `base_frame`: The base frame of the turtlebot.
    - `odom_frame`: The odometry frame of the turtlebot.
    - `map_frame`: The map frame of the turtlebot.
- Remap some topics to synchronize the SLAM process for the two turtlebots.
    - From `/map` to `/ns/map`.
    - From `/scan` to `/ns/scan`.
    - From `/initialpose` to `/initialpose_ns`.

### multi_robot_map_merge.launch

- Initialize the following parameters:
    - `known_initial_pos`: If the initial position of the robots is known.
    - `first_tb3`, `second_tb3`: The name of the two turtlebots.
    - `first_tb3_x_pos`, `first_tb3_y_pos`, `first_tb3_z_pos`, `first_tb3_yaw`: The initial position of the first turtlebot.
    - `second_tb3_x_pos`, `second_tb3_y_pos`, `second_tb3_z_pos`, `second_tb3_yaw`: The initial position of the second turtlebot.
- Create two groups with the namespace `first_tb3/map_merge` and `second_tb3/map_merge` with the following parameters.
    - `init_pose_x`, `init_pose_y`, `init_pose_z`, `init_pose_yaw`: The initial position of the turtlebot.
- Run the node [`map_merge`](#map_merge) to merge the maps of the two turtlebots.
    - `robot_main_topic`: The main topic of the turtlebot.
    - `robot_namespace`: The namespace of the turtlebot.
    - `merged_map_topic`: The topic to publish the merged map.
    - `world_frame`: The world frame.
    - `known_init_poses`: If the initial position of the robots is known.
    - `merging_rate`: The rate to merge the maps.
    - `discovery_rate`: The rate to discover the maps.
    - `estimation_rate`: The rate to estimate the maps.
    - `estimation_confidence`: The confidence to estimate the maps.
- Remap some topics to merge the maps of the two turtlebots.
    - From `first_tb3/map` to `new_tb3_0_map`.
    - From `second_tb3/map` to `new_tb3_1_map`.
- Run some `static_transform_publisher` nodes to publish the transformation between the two turtlebots and the map frame.
    - From `/map` to `/first_tb3/map`.
    - From `/map` to `/second_tb3/map`.
    - From `/map` to `/new_tb3_0_map`.
    - From `/map` to `/new_tb3_1_map`.

### modified_move_base.launch

- Initialize the following parameters:
    - `ns`: The namespace of each turtlebot.
    - `robot_base_frame`: The base frame of the turtlebot.
    - `move_forward_only`: If the turtlebot can only move forward.
- Run the node `move_base` to start the navigation process for the two turtlebots.
    - `base_local_planner`: The local planner for the turtlebot.
    - Initialize `yaml` parameters for the local planner.
- Remap some topics to start the navigation process for the two turtlebots.
    - From `robot_base_frame` to `/ns/base_footprint`.
    - From `/map` to `/ns/map`.

### tb3_*.launch

- Initialize the following parameters:
    - `init_pose`: The initial position of the first turtlebot.
    - `robot_name`: The name of the first turtlebot.
- Define the `robot_description` parameter for the first turtlebot.
- Run the node `robot_state_publisher` to publish the state of the first turtlebot.
    - `publish_frequency`: The frequency to publish the state of the turtlebot.
- Run the node `spawn_model` to spawn the first turtlebot.

## NODE FILES

### map_node

The `map_node` is a node from the `multi_robot_exploration` package that is used to read the robots local SLAM map and create a larger map to be used for the `map_merge` node.

```mermaid
classDiagram
    direction LR
    OccupancyGrid <|-- Header
    OccupancyGrid <|-- MapMetaData
    Header <|-- time
    MapMetaData <|-- Pose
    MapMetaData <|-- time
    Pose <|-- Quaternion
    Pose <|-- Point

    OccupancyGrid : Header header
    OccupancyGrid : MapMetaData info
    OccupancyGrid : int8[] data
    class Header{
      string frame_id
      int32 seq
      time stamp
    }
    class MapMetaData{
      time map_load_time
      Pose origin
      float32 resolution
      uint32 width
      uint32 height
    }
    class time{
      int32 nsecs
      int32 secs
    }
    class Pose{
        Quaternion orientation
        Point position
    }
    class Quaternion{
        float64 w
        float64 x
        float64 y
        float64 z
    }
    class Point{
        float64 w
        float64 x
        float64 y
        float64 z
    }
```

**Figure 1:** The `nav_msgs/OccupancyGrid` message structure.

![map_expanding](img/map_expanding.png)

**Figure 2:** The map expansion process.

Pseudo code for the `map_node`:
```
CREATE slam0_map, slam1_map (nav_msgs/OccupancyGrid)

map0callback:
    SET slam0_map : msg

map1callback:
    SET slam1_map : msg

main:
    INITIALIZE map_expansion_node

    PUBLISH new_tb3_0_map, new_tb3_1_map
    SUBSCRIBE tb3_0/map CALLBACK map0callback
    SUBSCRIBE tb3_1/map CALLBACK map1callback

    WHILE ros::ok
        IF (slam0_map OR slam1_map) are NOT empty
            CREATE new_tb3_0_map, new_tb3_1_map (nav_msgs/OccupancyGrid)

            SET new_tb3_0_map, new_tb3_1_map:
                header.frame_id = "new_tb3_*_map"
                info.resolution = 0.05
                info.origin.position.x = -10.0
                info.origin.position.y = -10.0
                info.origin.position.z = 0.0
                info.origin.orientation.w = 0.0
                info.width = 384
                info.height = 384
            
            FILL new_tb3_0_map, new_tb3_1_map
                FROM (0, 0) TO (width, bottom*_height) WITH -1
                FROM (0, bottom*_height) TO (bottom*_width, bottom*_height+height) WITH -1
                FROM (bottom*_width, bottom*_height) TO (bottom*_width+width, bottom*_height+height) WITH slam*_map
                FROM (bottom*_width+width, bottom*_height) TO (width, bottom*_height+height) WITH -1
                FROM (0, bottom*_height+height) TO (width, height) WITH -1 
        
            PUBLISH new_tb3_0_map, new_tb3_1_map
        
        ros::spinOnce
```

### tb3_0_FE

The `tb3_0_FE` is a node from the `multi_robot_exploration` package that is used to explore the environment and find the frontiers for the first turtlebot.

```mermaid
classDiagram
    direction LR
    class FrontExpl{
        NodeHandle nh
        Publisher FE0_map_pub
        Subscriber map_0_sub
        ServiceServer start_srv
        Buffer tfBuffer
        OccupancyGrid FE0_map
        Pose robot_pose_
        Point point
        TransformStamped transformS
        MoveBaseGoal goal
        string map0_frame = "tb3_0/map"
        string body0_frame = "tb3_0/base_footprint"
        vector<signed int> edge0_vec, neighbor0_index, neighbor0_value
        vector<unsigned int> centroids0, temp_group0
        vector<double> centroid0_Xpts, centroid0_Ypts, dist0_arr, prev_cent_0x, prev_cent_0y
        int group0_c=0, prev_group0_c=0
        int centroid0=0, centroid0_index=0
        int map_width=0, map_height=0
        int mark_edge=0, edge_index=0
        int move_to_pt=0
        double smallest = 9999999.0, dist0=0.0
        bool unique_flag=true
        bool start0_flag=false

        FrontExpl()
        bool startCallback()
        void mapCallback()
        void neighborhood()
        void find_all_edges()
        bool check_edges()
        void find_regions()
        void find_transform()
        void centroid_index_to_point()
        void find_closest_centroid()
        void edge_index_to_point()
        void main_loop()
    }
```
**Figure 3:** The `tb3_0_FE` class structure.

Pseudo code for the `tb3_0_FE`:

Class **FrontExpl**

**FrontExpl**
1. Publish the **edges_map_0** topic
2. Subscribe to the **map** topic with the **mapCallback** callback
3. Create a service server **start_srv** with the **startCallback** callback

**mapCallback**
1. Set the *FE0_map* to the received message
2. Change its *frame_id* to "edges_map_0"

**startCallback**
1. Set the *start0_flag* to true

**main_loop**
1. Define a MoveBaseClient object *ac* to "tb3_0/move_base"
2. Wait for the *move_base* action server to come up
3. Define a TransformListener object *tfListener*
4. Enter the while loop
    1. If the *FE0_map* is not empty and the *start0_flag* is true
        1. Obtain the map width and height from the *FE0_map*
        2. Find the frontier edges with the **find_all_edges** function
        3. If there are not any edges, *edge0_vec* is empty
            1. Go to **skip**
        4. Sort the edges in ascending order and remove duplicates
        5. Publish the *FE0_map* to the **edges_map_0** topic
        6. Given the edges, find the regions with the **find_regions** function
        7. Find the transform between the map and the robot with the **find_transform** function
        8. Given the centroids, convert them to points with the **centroid_index_to_point** function
        9. If there are not any centroids, *centroids0_Xpts* and *centroids0_Ypts* are empty, go to the closest frontier point
            1. Clear both *centroids0_Xpts* and *centroids0_Ypts*
            2. Clear the *dist0_arr* vector
            3. Given the edge vector, convert them to points with the **edge_index_to_point** function
        10. Of the centroids, find the closest one to the robot with the **find_closest_centroid** function
        11. Add the current centroid to the *prev_cent_0x* and *prev_cent_0y* vectors
        12. Determine the move_base goal with a frame of "tb3_0/map" and a position of the current centroid
        13. Send the goal to the move_base action server
        14. Wait for the result of the goal
    2. **skip**:
        1. Sleep for 1 second
        2. Continue to the next iteration
        3. Resetting all the variables and vectors

**find_all_edges**
1. Iterate through the *FE0_map* from the inside bottom right corner to the inside top left corner
    1. If the cell is unknown
        1. Check neighbors with the **neighborhood** function
        2. Iterate through the neighbors
            1. If the neighbor is free, its value is 0
                1. Store the index of the neighbor in the *edge0_vec* vector

**neighborhood** Given *cell* index
1. Clear the *neighbor0_index* and *neighbor0_value* vectors
2. Store the index of the neighbors, 8 in total, in the *neighbor0_index* vector
3. Sort the neighbors in ascending order and remove duplicates

**find_regions**
1. Iterate through the *edge0_vec* vector, which contains the frontier edges
    1. If the next edge is the same as the current edge
        1. Check the neighbors with the **neighborhood** function
        2. Iterate through the neighbors
            1. If the neighbor is a frontier edge
                1. Store the corresponding frontier edge in the *temp_group0* vector
                2. Sort the *temp_group0* vector in ascending order and remove duplicates
                3. Increment the *group0_c* counter to keep track of the region's size
        3. If we didn't add any new edges to the region, *group0_c* = *prev_group0_c*
            1. If the frontier region is big enough, *group0_c* >= 5
                1. Find the region's centroid, the middle point of the *temp_group0* vector
                2. Store the centroid in the *centroids0* vector
            2. Reset the *temp_group0* vector and *group0_c* counter
        4. Else, *prev_group0_c* = *group0_c*

**find_transform**
1. Get the transform between the *map0_frame* and the *body0_frame* in the *transformS* variable
2. Update the transform's stamp, frame_id, and child_frame_id
3. Update the robot's position and orientation from the *transformS* variable

**centroid_index_to_point**
1. Iterate through the *centroids0* vector
    1. Convert the centroid index to a point, using the map width and height
    2. Store it in the *x* and *y* elements of the *point* variable
    3. Iterate through the *prev_cent_0x* vector
        1. Calculate the distance between the current centroid and the previous centroids
        2. If the distance is too small, go to **bad_centroid**
    4. If the centroid is to close to the origin of the map, go to **bad_centroid**
    5. Else
        1. Store the *x* and *y* elements in the *centroids0_Xpts* and *centroids0_Ypts* vectors
        2. Determine the euclidean distance between the robot and the centroid
        3. Store the distance in the *dist0_arr* vector
    6. **bad_centroid**:
        1. Print a message that the centroid is not valid

**find_closest_centroid**
1. Set the *smallest* variable to a large number
2. Iterate through the *dist0_arr* vector
    1. If the distance is too small, ignore it
    2. If the distance is smaller than the *smallest* variable
        1. Update the *smallest* variable
        2. Update the index of the centroid to the *move_to_pt* variable
    3. If the distance is larger than the *smallest* variable, ignore it

**edge_index_to_point**
1. Iterate through the *edge0_vec* vector
    1. Convert the edge index to a point, using the map width and height
    2. Store it in the *x* and *y* elements of the *point* variable
    3. Store the *x* and *y* elements in the *centroids0_Xpts* and *centroids0_Ypts* vectors
    4. Determine the euclidean distance between the robot and the edge
    5. Store the distance in the *dist0_arr* vector

**main**
1. Initialize ROS node with name "tb3_0_FE"
2. Define a FrontExpl object *FE*
3. Run the **main_loop** function of the *FE* object


### tb3_1_FE

### map_merge

### sync_slam_toolbox_node