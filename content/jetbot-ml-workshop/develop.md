---
title: "Activity #1: An Introduction to ROS Development"
chapter: true
weight: 2
description: In this activity you will clone the Jetbot sample code and install the dependencies required.
---
# An Introduction to ROS Development

In the first activity, you will get familiar with ROS and learn how to build and bundle an application that you can use in an AWS RoboMaker simulation. Then, you will launch a simulation job with that application. Effectively testing ROS applications through simulations is hard. Robotics developers do not always have access to physical equipment (cost, size etc.) and would not be able to reproduce all of the real-world environments their robots may encounter. You can also do interesting things like train a machine learning model in simulation. 

AWS RoboMaker makes it easy to run your ROS application in a cloud-based simulation. When you run an AWS RoboMaker simulation job, you have access to 4 core tools to interact with your running application:

- **Gazebo** lets you build 3D worlds with robots, terrain, and other objects. It also has has a physics engine for modeling illumination, gravity, and other forces. Robotics developers use Gazebo to evaluate and test robots in different scenarios, often times more quickly than using physical robots and scenarios. Gazebo also makes it easier to test other aspects of your robot like error handling, battery life, navigation, and machine learning algorithms.
- **rviz** is a 3d visualization tool for ROS applications. It provides a view of your robot model, capture sensor information from robot sensors, and replay captured data. It can display data from camera, lasers, from 3D and 2D devices including pictures and point clouds.
- **rqt** hosts a number of different plugins for visualizing ROS information. Multiple plugins can be displayed on a custom dashboard, providing a unique view of your robot. rqt includes many useful plugins and provides a framework to write custom plugins.

Finally, you have a **terminal** to run commands in the simulation job. First, let's setup our development environment

### Setup your Development Environment

1. Open a new tab to the AWS RoboMaker console (*Services->RoboMaker->right-click->new tab*)

2. Create a development environment (*Development->Development environments->Create environment*) and complete the following:

    * Name: `workshop` or something descriptive
    * Instance type: `m4.large`
    * Select `ROS Melodic` as the pre-installed ROS distribution.
    * Choose the VPC (default), and a subnet for your development environment 
    * Click **Create**

3. This opens the environment's detail page, click *Open environment*, which will open a new browser tab with the Cloud9 IDE.

    *This may take a few minutes to complete, but when the creation process has completed, you will see something similar to this:*

    ![1_cloud9](../../images/1_cloud9.png)

    The *Welcome page* provides helpful information to get started, but for now we are not going to use it, so click the *X* on the tab to close. Take a look at the `roboMakerSettings.json` file. This contains the configurations (such as which S3 buckets and IAM roles) to use with AWS RoboMaker. The IDE is broken down into four sections:

    ![1_c9_layout](../../images/1_c9_layout.png)

    - (1) The AWS RoboMaker menu provide quick access to common actions. It is updated when the `roboMakerSettings.json` is modified later in this task.
    - (2) Any files and folders will reside here, and can be selected and double-clicked to open in the editor pane (#4).
    - (3) The lower section is an adjustable pane for creating or monitoring command line operations. ROS developers work in this area to build, test, and interact with local code.
    - (4) This is the main editor pane.

---

# STOP

New Cloud9 instances take some time to provision and update themselves.  If you proceed at this point you may have issues running the scripts to install dependencies. 

---

### Clone the Robot Application and Install Dependencies

The first step is to open the AWS RoboMaker IDE and clone the AWS-enabled JetBot ROS application. 

1. Open the AWS CloudFormation console at https://console.aws.amazon.com/cloudformation/

1. In CloudFormation console and write down the CloudFormation name that looks similar to mod-xxxxxxx36e49dc. You will need this to run an automation script later to set up RoboMaker IDE enviornment. 
![cloudformation name](../../images/cloudformation-name.png)

1. Open the RoboMaker IDE and navigate to the terminal and clone this Git repo to the development machine: 
    ``` 
    # change to the environment directory 
    $ cd ~/environment 
    $ git clone https://github.com/jerwallace/simple-road-following-app.git jetbot
    ``` 

 1. In the terminal window, change to the `jetbot/assets` directory and run the install script.

    ``` 
    # Run install_dep.sh to install prerequisite 
    $ cd ~/environment/jetbot/assets/scripts 
    $ chmod +x compile_arm64.sh install_deps.sh configure_docker.sh
    $ sudo ./install_deps.sh <your CloudFormation name>
    ``` 

    - **What is happening here?** This shell script (install_deps.sh) is going to setup a few key dependencies for us today. It will:
      - Generate a *set of x.509 certificates* to connect with **AWS IoT** over **MQTT**
      - Configure the *roboMakerSettings.json* file with the right **IAM permissions** and an **Amazon S3** bucket to store your ROS applications and logs.
      - Add a new custom ROS sources list to include some of the extra libraries we are using (such as the motor controller).

### Run in Simulation and Explore

1. In RoboMaker IDE menu, click **Run, Workflow, select JetBot Simulation - Build and Bundle** *(this process takes about 10 minutes)*. As the name suggests, it will compile/build the ROS application and bundle it into a tar file for simulation. 
   
    - **IMPORTANT NOTE: If you experience the error: "Unable to acquire the dpkg frontend lock", that is because when Cloud9 first loads, it runs an update command. This will take a few minutes. Not to worry, simply try again in a few minutes and the JetBot Simulation - Build and Bundle will run successfully.**
 
    - **Why are we doing this?** There are a couple of build tools that developers use with ROS. The one that we will use with AWS RoboMaker is called [colcon](https://colcon.readthedocs.io/en/released/). However, in addition to your application files, you will also need to bundle your application with the necessary dependencies. This includes any libraries you are using in your ROS application as well as the system dependencies. The tool [colcon bundle](https://github.com/colcon/colcon-bundle) collects all of these dependencies as well as your built application and packages them up in an easy-to-deploy **.tar** file. A **workflow** is simply a set of shell commands (ex: `colcon build` and `colcon bundle`) that the IDE will run when you use the dropdown menus. A preconfigured workflow is included with the sample application in the *roboMakerSettings.json* file.

2. Make sure the Colcon Bundle tab displays "Process exited with code: 0". This indicates the build and bundle process has completed.

![Simulation menu](../../images/simulation-menu.png)

3. Click **Run, Launch Simulation, JetBot Circle Simulation**. This will launch the application in simulation enviornment with the Jetbot rotating in circles.

4. The status next to Simulation should change to (Pending) indicating that the simulation has started, and then will finally go to (Running) when it has completed.  If this has not updated after a few minutes, please try refreshing your browser or going to the Robomaker Console > Simulation Jobs to see the up-to-date status.

5. When simulation status displays (running), explore the enviornment in Gazebo by clicking on the Simulation menu, Applications, Gazebo. Use your mouse scroll wheel to zoom in and out of the enviornment. This simple application demonstrates that you have all the components configured and installed correctly. Now you can move on to the next simulation, teleop, which allows you to remote control the JetBot. 

![simulation gazebo](../../images/simulation-gazebo.png)

6. Stop the simulation from Simulation (Running) menu

7. In RoboMaker IDE menu, click Run, Launch Simulation, JetBot Teleop Simulation - This will launch the application in simulation enviornment where you can drive the Jetbot with the teleop client app. When simulation status displays (running), explore the enviornment in Gazebo by clicking on the Simulation menu, Applications, Gazebo. Use your mouse scroll wheel to zoom in and out of the enviornment. Be sure to continue with the following steps to download the joy stick client application to remote control the JetBot.

### Optional Fun Exercise: Run Teleop in Simulation

1. Locate the teleop.zip file in **jetbot/assets/teleop** folder 

![teleop app](../../images/teleop-app.png)

1. Download the zip file in the file explorer and unzip it on the desktop
1. Open the **robogui.html** file in a browser and make sure the connection status states `Connected`
1. Use your mouse to move the joy stick and drive the Jetbot in Gazebo 
1. Stop the simulation from Simulation (Running) menu

### Clean-up Reminder

**Note for those running in their own AWS accounts and are not using a workshop code:**

In this activity, you created a some resources (Development environment, CloudWatch logs, and S3 objects) that incure cost. If you **are not continuing** on with the next sections of the workshop, remember to go to the [clean-up steps](https://www.robomakerworkshops.com/workshop/cleanup/) and remove these resources to stop any potential costs for occurring.
