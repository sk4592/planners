#!/bin/bash

echo "hello world"

function clean_processes {
    processToKill=(rviz2 gzserver gzclient ros2 component_conta python3 robot_state_pub)
    # allProcessess = ps -a 
    for process in ${processToKill[@]}; do
        echo $process
        for pid in $(ps -ef | grep $process | awk '{print $2}'); do
            kill -9 $pid; 
        done
    done
}