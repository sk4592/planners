### Running map server as lifecycle node ###

ros2 run nav2_map_server map_server --ros-args --params-file params.yaml

params.yaml should consist of map.yaml
example: 
    map_server:
    ros__parameters:
        yaml_filename: "turtlebot3_world.yaml"
        

### call lifecycle node in another terminal for transitioning state ###

# configure 
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}" 

# activate
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}" 