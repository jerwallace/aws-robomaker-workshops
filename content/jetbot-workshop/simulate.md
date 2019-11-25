---
title: "Activity #2: Teleop in Simulation"
chapter: true
weight: 2
description: "In this activity you will build, bundle, and run the simulation and teleop the robot around in a simulation world."
---

# Teleop in Simulation

### Run in Simulation and Explore
 
1. Open the roboMakerSetting.json file in the **jetbot** directory and input S3 bucket, IAM role, MQTT endpoint and VPC public subnet ids and security group ids. 
 
1. Click Run, Add or Edit Configurations, select the roboMakerSettings.json file from jetbot directory 
 
1. Click Run, Workflow, select JetBot Simulation - Build and Bundle (this process takes about 10 minutes) 
 
1. Click Run, Launch Simulation, JetBot Circle Simulation - This will launch the application in simulation enviornment with the Jetbot rotating in circles. When simulation status displays (running), explore the enviornment in Gazebo. 
 
1. Stop the simulation 
 
1. Click Run, Launch Simulation, JetBot Teleop Simulation - This will launch the application in simulation enviornment where you can drive the Jetbot with the teleop client app. When simulation status displays (running), explore the enviornment in Gazebo. 
 
1. Zip the teleop client app 
    ``` 
    # Make sure you are in the jetbot directory 
    $ cd ~/environment/jetbot 
    $ zip teleop.zip assets/teleop/* 
    ``` 
1. Download the zip file in the file explorer and unzip it on the desktop 
1. Open the robogui.html file in a browser and make sure the connection status states Connected 
1. Use your mouse to move the joy stick and watch the Jetbot move in the Gazebo window 
1. Stop the simulation 
