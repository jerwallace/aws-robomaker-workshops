---
title: "Activity #1: Open Source Rover"
chapter: true
weight: 5
description: "In this activity you will setup a development environment and build, bundle, and simulate a Mars rover in an open world."
---

# Simulate a Open Source Rover

![rover](../../images/open-source-rover/Curiosity-Power.jpg)

Our objective: Move the rover in simulation

In this exercise, we'll use AWS RoboMaker to simulate a Open Source Rover operating in a Martian environment.  First you'll create a simulation in an empty environment and learn how to connect to the robot while it's running in simulation.    

When complete, you will have learned:

* How to use AWS RoboMaker to build ,package (bundle), and simulate ROS applications.
* How to create a simulation job.
* How to interact with your robot while it's running in simulation.

## Activity tasks

1. Navigate to the [AWS RoboMaker console](https://us-west-2.console.aws.amazon.com/robomaker/home?region=us-west-2#welcome).  Ensure that the region is set to *US West (Oregon)*.

2. Create a development environment where we can develop our ROS application.  Click the hamburger menu on the upper-left of the page to expand the RoboMaker menu.  Then create a new development environment by navigating to *Development->Development environments* and choose **Create environment**.
 
    ![create-env](../../images/open-source-rover/create-environment.jpg)
 
  
3. On the *Create AWS RoboMaker development environment* page, enter the following:

    * Name: `rover-workshop` or something descriptive
    * Pre-Installed ROS Distribution: `ROS Kinetic`
    * Instance type: `m4.large`
    * Choose the VPC that was just created by the CloudFormation stack.  You will see two VPCs.  Choose the one with the name of your CloudFormation stack.  This will NOT be the *Default* VPC - choose the other VPC. 
    * Subnets: Choose either of the two subnets.
    * Click **Create**

4. This opens a new tab in your browser with the Cloud9 IDE.

    *This may take a few minutes to complete, but when the creation process has completed, you will see something similar to this:*

    ![1_cloud9](../../images/1_cloud9.png)

    The *Welcome page* provides helpful information to get started, but for now we are not going to use it, so click the *X* on the tab to close.  The IDE is broken down into four sections:

    ![1_c9_layout](../../images/1_c9_layout.png)

    - (1) The AWS RoboMaker menu provide quick access to common actions. It is updated dynamically when a file in your project is modified.  The `roboMakerSettings.json` file is used to configure your RoboMaker project, and drives the content of this menu.
    - (2) Any files and folders will reside here, and can be selected and double-clicked to open in the editor pane (#4).
    - (3) The lower section is an adjustable pane for creating or monitoring command line operations. ROS developers work in this area to build, test, and interact with local code.
    - (4) This is the main editor pane.

5. Delete the `roboMakerSettings.json` file by right-clicking on it and selecting *Delete*->Yes. We are going to clone a new project from GitHub, and it will contain this file.  So we can safely delete the default file.

6. For this activity, you will be using a command line terminal to perform several tasks associated with the development of our robot.  Throughout this guide, we'll reference *the Bash shell*.  When you see that, we want you to run commands in the bash shell in your Cloud9 environment (Section 3 of the IDE, as defined in the previous step).

7. The project we'll be working with is located in GitHub.  You need to clone it into the Cloud9 environment so you can work with it.  Copy and paste the following commands into the Bash shell.  This will clone the repository:

    ```text
    cd ~/environment

    # clone the Open Source Rover repository
    git clone https://github.com/aws-robotics/aws-robomaker-sample-application-open-source-rover.git
    ```

8.  You will now have a new directory in your project called *open-source-rover*.  Let's take a look at the contents of that folder.  There are two folders of interest in our project.  The first folder, *content* is not related to our robot code.  This contains some HTML and JavaScript files that will be used later to build a dashboard to view our robot metrics.  We'll come back to that later.  Expand the aws-robomaker-sample-application-open-source-rover folder (you should now be looking at *your-top-directory->aws-robomaker-sample-application-open-source-rover*).  In this folder, we see two new folders called *robot_ws* and *simulation_ws*.  These are the workspaces for our robot application.  In ROS development, a workspace is a folder where you modify, build, and install packages.  It is common practice for robotics developers to create multiple workspaces for their system to better encapsulate the components.  In our workshop, we have a workspace for our robot code (robot_ws).  This contains all the ROS nodes, services, and any dependencies needed by the robot application.  We also have a second workspace for our simulation material (simulation_ws).  A simulation workspace typically contains the artifacts needed to run our robot in simulation.  This contains items such as the 3D model for the robot and the 3D objects and textures needed to create the world in which the robot will be simulated.  In this workshop, we're going to be working in these folders to view and modify our robot code.  

    ![roboMakerSettings](../../images/open-source-rover/nav-tree.png)

    We'll review much of the code in more detail in the next activity.  Before we do that, let's configure and run a simulation.

9.  Before we can run any simulations, we need to configure the environment to use the networking and roles we created in the earlier setup activity.  To do this, edit the project settings by using the menu at the top of the IDE and navigating to *Run->Add or Edit Configurations...*.  This will open the configuration window.

    ![config](../../images/open-source-rover/configuration.jpg)
     
10.  We need to tell RoboMaker what file to use to save our project configuration.  Click the **Switch config** button on the lower-left side of the window and navigate to the file *aws-robomaker-sample-application-open-source-rover/roboMakerSettings.json*.  Click **Ok**.

    ![select-config](../../images/open-source-rover/select.png)

    Click **Save** to close the window.  We need to save this change before we make any more edits to the configuration.

11. Re-open the configuration window from the IDE menu *Run->Add or Edit Configurations...*.  Expand the *Simulation* item in the left-navigation area and click on *Open Source Rover Simulation*.

12. Let's configure the simulation job to use an IAM role that was created earlier.  When the simulation runs, it will assume this role.  This gives the robot application the permissions it needs to access other AWS resources during simulation.  To set it, scroll until you find the property **Simulation job->IAM role**.  In the drop-down, choose the role named *robomaker-simulation-role*.

14.  Finally, let's configure the simulation project to use the S3 bucket we created earlier during setup.  Note that there are **three** locations in the configuration where we need to set the S3 bucket. The bucket was already created by the CloudFormation stack you launched. The bucket name we will use is similar to: `<CloudFormationStackName>-us-west-2-rmw-assets`.  If you didn't note the bucket name, you can find it as the RoboMakerS3Bucket key in the **Outputs** section of the CloudFormation stack.
  - The first location is located at **Robot application->S3 bucket**.  In this field, click the drop-down and select the bucket created earlier.
  - The second location is located a bit further down under **Simulation application->S3 bucket**. Choose the same bucket from the drop-down list. 
  - The final location is near the bottom of the file in **Simulation Job->output location**. Choose the same bucket from the drop-down list.

15.  Before we exit the configuration, let's take a look at some of the other properties you can set.  We're not going to change these today, but let's review for future use.  In the menu on the left, notice the menu items for COLCON BUILD and COLCON BUNDLE.  Colcon is a tool that is used to compile (build) and package (bundle) ROS workspaces.  Our configuration has been pre-configured to build and bundle the workspaces used in the workshop.  Also notice the WORKFLOW menu item.  This enables you to customize any build and bundle operations you may need in your project.  For today's workshop, we have created three workflows: one that will build and bundle all the workspaces, another that builds and bundles only the robot application, and a third workflow that builds and bundles only the simulation workspace.

16.  Those are all the changes we need for now.  Click **Save** to exit the configuration window.  

17.  Ok, enough configuring already.  Let's build and bundle our robot application and see it running in simulation!  To build and bundle our robot application and simulation material, use the IDE menu and choose *Run->Workflow->Open Source Rover - Build and Bundle All*.  This will kick-off the compilation and packaging of our robot and simulation workspaces.  This will take about five minutes to complete.  Subsequent build and bundle operations usually complete faster, but the first time you build and bundle, it will download dependent packages which will take a few minutes.  While this operation is executing, you'll see two new tabs open in the Bash shell area of the IDE.  You can look at these tabs to see the log outputs of the build and bundle operations.  When the process is complete, the last few log entries in the *Colcon Bundle* tab will display something similar to:

    ```text
    Fetched 473 MB in 6s (3468 kB/s)                                               
    Extracting apt packages...
    Creating bundle archive V2...
    Archiving complete!
    Process exited with code: 0
    ```

    The final bundling step for the simulation workspace may take upwards of 5 minutes.  Please be patient.  If you see it stuck an output like, `Creating bundle archive V2...` then it's in the final step of creating the output bundle.  The resulting output of these operations is two .tar files containing our robot application, and our simulation artifacts.  For reference, the files for the robot workspace and the simulation workspace are saved to `aws-robomaker-sample-application-open-source-rover/robot_ws/bundle/output.tar` and `aws-robomaker-sample-application-open-source-rover/simulation_ws/bundle/output.tar` respectively.  If you do not see both of these files, then your workspaces haven't yet finished building and bundling.  Please wait until both files exist before moving to the next step.   

18.  Kick off the simulation job by using the IDE menu and choosing *Run->Launch Simulation->Open Source Rover Simulation*.  You'll notice another tab open in the Bash shell section of the IDE.  You can witness the IDE copying the .tar files mentioned above to S3, and then the simulation job will be created.  In the IDE menu, you'll see the menu indicate that the simulation is being prepared.

    ![sim-preparing](../../images/open-source-rover/sim-preparing.jpg)
 
    When the simulation is ready and running, the menu will change to *Running*.

19. Once it is running, we can now interact with the simulation.  AWS RoboMaker includes several common robot simulation tools to interact with your robot.  In this step, we'll use the open source tool [Gazebo](http://gazebosim.org/) to interact with the robot.  In the IDE menu, choose *Simulation (Running)->Applications->Gazebo*.  This will open a new window where you will see the Open Source Rover in a Martian environment.
 
    ![rover-world](../../images/open-source-rover/gazebo.png)

    If your browser asks you to allow or block access to the clipboard, click **Allow**.  This will allow you copy and paste commands.
    
    ![gazebo-allow](../../images/open-source-rover/gazebo-allow.jpg)

    You can change the zoom level and view angle to obtain a better view of the rover and the environment.  Note, that this is *much* easier to do with a mouse rather than a laptop touchpad.  You can zoom with a mouse wheel (our touchpad equivalent).  To adjust the positioning of the view, click and drag in the world.  To adjust the angle of view, shift-click and drag.  If you have issues and you lose sight of the rover, you can reset the view using the Gazebo menu.  To reset, click *Camera->Reset View Angle*.

20.  When a robot is running in the simulation environment, we can connect to it to inspect it, control it, and debug it.  To do this, we're going to use a terminal that will be connected to the robot application in the simulation.  From this terminal, we'll be able to view the ROS topics and messages, as well as send specific commands to the robot.  To open the terminal, go back to the Cloud9 IDE menu, and choose *Simulation (Running)->Applications->Terminal*.  This will open a new window containing a terminal connected to the robot.  Adjust your Gazebo window and your Terminal window so you can see each comfortably.  Also adjust the view in Gazebo so you can see the rover in a wider landscape.  This will make it easier to see the robot while it moves.

    ![gazebo-with-terminal](../../images/open-source-rover/gazebo-with-terminal.png)

    Let's take a look at some of the ROS information for our robot.  To see all the ROS topics that are being published, issue the following command in the terminal:
    
    ```text
    rostopic list
    ```

    *Note*: To paste commands in the Terminal window, use `Shift+Ctrl+V`, or use the *Edit->Paste* menu item.

    You will see a long list of topics.  One of the topics, `/rosout`, contains logging information from other nodes in the system.  To see the log data currently being generated by the robot, issue the following command in the Terminal:
    
    ```text
    rostopic echo /rosout
    ```

    You'll see lots of data quickly scrolling through the terminal.  When you've had enough, stop echoing the topic by pressing `Ctrl-C` in the terminal.

21.  Let's make this thing move!  We're going to use our keyboard to control the robot in the simulation environment.  To do so, we'll run a script that has been deployed with our robot that takes keyboard input and converts it to motion-control messages that are sent to the robot.  To run this script, use the same terminal window from the previous step and issue the following command:

    ```text
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
    
    Use the keys displayed in the terminal to move the robot. For example to move forward, press `i`, to move clockwise, press `o`.  You don't need to press and hold - you only need to press the key once to send that instruction to the robot.  By default, the rover will move rather slowly.  Pres the `q` key on your keyboard several times to increase the speed.  Increase the speed to approximately 6, and then issue another forward (`i`) command to see it move faster.  Continue experimenting with the controls to move the rover.  The `teleop_twist_keyboard` script publishes messages of type *Twist* to the */cmd_vel* topic.  Both the *Twist* message type and the */cmd_vel* topic are commonly used by robot developers for motion control. 
    
    Did you know that the gravity on mars is only about 1/3 of that on earth?  In the Gazebo simulation, the gravity has been reduced to simulate Mars.  If you try to corner your rover too quickly, you can see the results of low gravity and angular momentum.  Try increasing the speed to about 18 to 20, and then attempt to turn (`o`) your rover.  What happens?

    When you've had enough of driving the rover around Mars, you can stop the simulation using the IDE menu, *Simulation (Running)->Stop*. 

## Activity wrap-up

In this activity, you used AWS RoboMaker to build and simulate a robot that you can control with a keyboard controller.  You viewed some of the ROS information for your robot, and you learned how to interact with the robot in the simulation environment.  

In the next activity, you'll extend the robot application to use AWS AI services to detect Martians in a new world! 

**[Proceed to the next activity](../martiandetector/)**
