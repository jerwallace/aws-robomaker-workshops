---
title: "Activity #1: Introduction to Jetbot Sample Code"
chapter: true
weight: 2
description: In this activity you will clone the Jetbot sample code and install the dependencies required.
---
# Introduction to Jetbot Sample Code

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

### Clean-up Reminder

In this activity, you created a Development environment, CloudWatch logs, and S3 objects that incure cost. If you **are not continuing** on with the next sections of the workshop, remember to go to the [clean-up steps](https://www.robomakerworkshops.com/workshop/cleanup/) and remove these resources to stop any potential costs for occurring.