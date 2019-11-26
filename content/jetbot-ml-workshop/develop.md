---
title: "Activity #1: An Introduction to ROS Development"
chapter: true
weight: 2
description: In this activity you will clone the Jetbot sample code and install the dependencies required.
---
# An Introduction to ROS Development

### Clone the Robot Application 
 
1. Open the RoboMaker IDE and navigate to the terminal and clone this Git repo to the development machine: 
    ``` 
    # change to the environment directory 
    $ cd ~/environment 
 
    $ git clone URL jetbot 
    ``` 
 
### Install Dependencies [~15 mins] 
1. Open the RoboMaker IDE and navigate to the terminal. 
 
1. In the terminal window, change to the `jetbot/assets` directory  
    ``` 
    # Run install_dep.sh to install prerequisite 
    $ cd ~/environment/jetbot/assets/scripts 
     
    $ chmod +x compile_arm64.sh install_deps.sh  
     
    $ sudo ./install_deps.sh 
     
    ``` 
1. Wait for previous step to complete and in the same terminal window, run the following command to update ROS dependencies  
    ``` 
    #  Make sure previous step is complete first 
    $ rosdep update 
     
    ``` 

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


### Clean-up Reminder

In this activity, you created a Development environment, CloudWatch logs, and S3 objects that incure cost. If you **are not continuing** on with the next sections of the workshop, remember to go to the [clean-up steps](https://www.robomakerworkshops.com/workshop/cleanup/) and remove these resources to stop any potential costs for occurring.