---
title: "Activity #3: Build & Bundle in ARM64"
chapter: true
weight: 3
requiredTimeMins: 15
description: In this activity you will explore other methods for building and deploying applications in the development environment including building for an ARM64 device using Docker.
---

# Build, Bundle and Deploy Robot Application in ARM64 Architecture
1. Open the RoboMaker IDE and navigate to the terminal

1. Change to the **jetbot** directory and build & bundle the ROS application in a docker container
    ```
    # Make sure you are in the jetbot directory
    $ cd ~/environment/jetbot
    
    # IMPORTANT: Make sure you are in the jetbot directory
    # Build and bundle the robot application
    $ docker run --rm -ti -v $(pwd):/environment/jetbot jetbot-ros

    # You will be dropped into the shell of the docker container
    # the prompt will be similar to the following root@83afb0b35322:/environment/jetbot# 

    (docker)$ ./assets/scripts/compile_arm64.sh 

    # Wait until shell script is completed 
    #Type exit or Ctrl-D, to exit the container
    (docker)$ exit
    ```

1. Back in the RoboMaker IDE and navigate to the terminal
    ```
    # Make sure you exited out of the container in previous step
    # Copy the robot application to S3
    $ aws s3 cp ./robot_ws/arm64_bundle/output.tar s3://<S3-BUCKET-NAME>/jetbot/aarch64/output.tar
    ```