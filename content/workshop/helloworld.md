---
title: "Activity #1: Hello Robot Simulation"
chapter: true
weight: 4
description: In this activity you will setup a development environment and build, bundle, and simulate a "Hello World" application.
---

# Development environment and HelloWorld

In this activity, you will setup a AWS RoboMaker development environment to quickly compile and run a "Hello World" ROS application. Once complete, you will have learned:

* How to navigate the AWS RoboMaker console and access development environment and simulation jobs
* Basic ROS workspace layout and build/bundle tasks
* How to submit and interact with a simulation job

## Activity tasks

1. Open a new tab to the AWS RoboMaker console (*Services->RoboMaker->right-click->new tab*)

2. Create a development environment (*Development->Development environments->Create environment*) and complete the following:

    * Name: `workshop` or something descriptive
    * Instance type: `m4.large`
    * Choose the VPC (default), and a subnet for your development environment 
    * Click **Create**

3. This opens the environment's detail page, click *Open environment*, which will open a new browser tab with the Cloud9 IDE.

    *This may take a few minutes to complete, but when the creation process has completed, you will see something similar to this:*

    ![1_cloud9](../../images/1_cloud9.png)

    The *Welcome page* provides helpful information to get started, but for now we are not going to use it, so click the *X* on the tab to close.The IDE is broken down into four sections:

    ![1_c9_layout](../../images/1_c9_layout.png)

    - (1) The AWS RoboMaker menu provide quick access to common actions. It is updated when the `roboMakerSettings.json` is modified later in this task.
    - (2) Any files and folders will reside here, and can be selected and double-clicked to open in the editor pane (#4).
    - (3) The lower section is an adjustable pane for creating or monitoring command line operations. ROS developers work in this area to build, test, and interact with local code.
    - (4) This is the main editor pane.

4. Delete the `roboMakerSettings.json` file by right-clicking on it and selecting *Delete*->Yes. We will use the example applications file to complete.

5. Next, use the menu to download and create the HelloWorld application by clicking *Resources->Download Samples->1. Hello World*. This will download, unzip, and load the readme file for the Hello World application.

6. While the application files are downloading, in another window, click on **AWS CloudFormation** and select the stack that you launched in the setup steps. In the **Outputs** you will find two role ARNs to use with RoboMaker. For this exercise, we will be using the *simulation role* which can be found in the key value pair titled 'SimulationRole' and the S3 bucket that was created. They should look similar to this:

    ```text
    arn:aws:iam::123456789012:role/robomaker-simulation-role
    <your-stack-name>-assets
    ```
  
    Copy the role ARN and the S3 bucket name and head back to your *RoboMaker Cloud9* editor.

7. For this project, we are going to use the menu option to build, bundle and simulate. Close the `README.md` file in the editor pane, then open the *HelloWorld* folder (double-click), and double-click the `roboMakerSettings.json` file to edit.

    This file contains all the settings to build the menu above. You will use these default settings, but need to complete the S3 bucket and IAM Role ARN  sections for your account.

9. Scroll down to the `simulation` section and replace the `output location` with your S3 bucket name. Below that, replace the `<your ... role ARN>` with the full ARN saved from the previous step. Save the file. This will refresh the menu options to use the new values.

    Just above the simulation attribute, also replace the s3Bucket entries for `robotApp` and `simulationApp` to match what was created in CloudFormation.

    *There are three locations to enter the S3 bucket details. If you receive an error when running, check to make sure all three are complete and the role ARN has been entered.*

    When done, the modified sections should look similar to this:

    ```json
       }, {
         "id": "HelloWorld_SimulationJob1",
         "name": "HelloWorld",
         "type": "simulation",
         "cfg": {
           "robotApp": {
             "name": "RoboMakerHelloWorldRobot",
             "s3Bucket": "<your-stack-name>-assets",
             "sourceBundleFile": "./HelloWorld/robot_ws/bundle/output.tar.gz",
             "architecture": "X86_64",
             "robotSoftwareSuite": {
               "version": "Kinetic",
               "name": "ROS"
             },
             "launchConfig": {
               "packageName": "hello_world_robot",
               "launchFile": "rotate.launch"
             }
           },
           "simulationApp": {
             "name": "RoboMakerHelloWorldSimulation",
             "s3Bucket": "<your-stack-name>-assets",
             "sourceBundleFile": "./HelloWorld/simulation_ws/bundle/output.tar.gz",
             "architecture": "X86_64",
             "launchConfig": {
               "packageName": "hello_world_simulation",
               "launchFile": "empty_world.launch"
             },
             "simulationSoftwareSuite": {
               "name": "Gazebo",
               "version": "7"
             },
             "renderingEngine": {
               "name": "OGRE",
               "version": "1.x"
             },
             "robotSoftwareSuite": {
               "version": "Kinetic",
               "name": "ROS"
             }
           },
           "simulation": {
             "outputLocation": "df-workshop",
             "failureBehavior": "Fail",
             "maxJobDurationInSeconds": 28800,
             "iamRole": "arn:aws:iam::YOUR_ACCOUNT:role/YOUR_ROLE"
           }
         }
       },
    ```

10. Next, update the hello world simulation application to use TurtleBot3 'waffle_pi' instead of 'burger'. The TurtleBot3 Waffle Pi has a camera module, which we will use as part of this activity. To do this, open the following file: 

    ```text
     HelloWorld/simulation_ws/src/hello_world_simulation/launch/empty_world.launch
    ```
    
    Then, add this line in the TurtleBot3 include block: `<arg name="model" value="waffle_pi" />`. After complete, the file should look like this:

    ```xml
      <launch>
        <!-- Always set GUI to false for AWS RoboMaker Simulation
            Use gui:=true on roslaunch command-line to run with a gui.
        -->
        <arg name="gui" default="false"/>

        <include file="$(find gazebo_ros)/launch/empty_world.launch">
          <arg name="world_name" value="$(find hello_world_simulation)/worlds/empty.world"/>
          <arg name="paused" value="false"/>
          <arg name="use_sim_time" value="true"/>
          <arg name="gui" value="$(arg gui)"/>
          <arg name="headless" value="false"/>
          <arg name="debug" value="false"/>
          <arg name="verbose" value="true"/>
        </include>

        <!-- Spawn Robot -->
        <include file="$(find turtlebot3_description_reduced_mesh)/launch/spawn_turtlebot.launch">
          <arg name="model" value="waffle_pi" />
        </include>
      </launch>
    ```
    

11. Now, use the menu to build and bundle both the robot and simulation application. Click *Run->Build->HelloWorld Robot* to start the compile for the robot application. This will take approximately 1-2 minutes as it needs to download and compile the code. When you see the successful process complete message `Process exited with code: 0`, use the same command to build the *HelloWorld Simulation*.

12. At this point both applications have been compiled locally. To run as a AWS RoboMaker simulation job, you also need to bundle them. In this step, the application along with all operating system dependencies are packaged in a bundle which is sort of like a container. After the process completes, a compressed output file will be generated locally. Similar to the build steps above, you need to bundle both the robot and simulation application. To do this, select *Run->Bundle->HelloWorld Robot*. **This will take 10-15 minutes or so to complete for both, and you may see Cloud9 warnings about low memory, which you can disregard.**

    While these are building, take a moment to review the JSON file for the simulation area. Here, you can see how the launch configs reference the package name (hello_world_robot or hello_world_simulation) and the specifc launch file to use (rotate.launch and empty_world.launch respectively). 

13. When both bundle operations are completed, launch a simulation job (*Run->Launch Simulation->HelloWorld*). This will do the following:

    * Upload the robot and simulation application bundles (approximately 1.2GiB) to the S3 bucket
    * Create a robot application and simulation application which reference the uploaded bundles
    * Start the simulation job in your defined VPC

    *Note: If you run into an error at this step, double check your `roboMakerSettings.json` file and ensure that all S3 references and the IAM role has been changed to the values from your CloudFormation outputs.

14. Open the AWS RoboMaker console and click on simulation jobs. You should see your job in a *Running* status. Click on the job id to see the values that were passed as part of the job. This view provides all the details of the job and access to tools which you will use in a moment.

     If the status shows Failed, it is most likely a typo or configuration issue to resolve. In the *Details* section, look for the *Failure reason* to determine what took place so you can correct.

15. From the simulation job details, we will launch a couple tools to interact with the robot. First, click on Gazebo, which will launch a pop-up window for the application. This is a client that provides a view into the virtual world.

     ![1_gazebo](../../images/1_gazebo.png)

     Using your mouse or trackpad, click into main window. Please refer to [this page](http://gazebosim.org/tutorials?tut=guided_b2&cat=) for more details and how to navigate (near the bottom of the page).

     When zoomed in, you will see robot turning slowing counter-clockwise in the same position. This is due to the `rotate.launch` file sent as the part of the robot application.

     Leave this window open for now and go back to the simulation job page. Here, click on Rviz, which will open a new window. :bulb: If possible, make the window larger on your laptop for better visibility.

     [Rviz](http://wiki.ros.org/rviz) is a 3D visualization tool for ROS. It provides information on the robot state and world around it (virtual or real). To get your virtual robot properly working, we need to point to a robot component. Click in the window, and then click on the "map" next to Fixed Frame:

     ![1_rviz_start](../../images/1_rviz_start.png)

     From the drop-down, select `base_footprint` and then click in the whitespace below. This should fix the *Global Status: Error* message. Next, select the Add button in the lower left, and from the *By display type*, select *Camera* and click *OK*. Notice the grid pattern in the camera window. To view what the virtual robot "sees", click the triangle next to Camera to twirl it down, then click into the Image Topic field. There should be a single topic name /camera/rgb/image_raw for you to select:

     ![1_rviz_camera](../../images/1_rviz_camera.png)

     Like above, once select click outside this area (or click on *Global Status: OK*) to have it take effect. The camera window should change to a two-tones gray view. This is correct as the world you launched in is truly empty. It's time to add something(s).

15. Navigate back to the Gazebo window and select on object (cube, ball, or cylinder) menu and then inside the world view drag you object near the robot and click to place. Do this a couple times to add objects around it.

     ![1_gazebo_objects](../../images/1_gazebo_objects.png) 

     And now go back to Rviz and look at the camera window. You will see the objects pass in front as the robot turns! If you can, position both windows so you can see the robot in Gazebo and the camera view from Rviz:

     ![1_both_apps](../../images/1_both_apps.png)

16. For the Hello World example, these are the basic operations of the robot and world. Before moving onto the next activity, let's take a look at some of the other things this simulation did.

17. First, close the Gazebo and Rviz windows and navigate back to the Simulation jobs page. From the simulation, under actions select *Cancel->Yes, Cancel* to stop the simulation. This gracefully terminates the simulation and stops charges. Go back to the list of simulation jobs and you should see the job in a *Cancelled* state. Click back into the job again, and under *Details* click on the *Simulation job output destination* link. This opens a new tab to S3 where this is folder corresponding to the simulation job ID. 

     Click into the folder, and then into the date/time stamped folder. Here you will three sets of outputs. The `gazebo-logs` folder contains the log files of the simulation applications output, while the `ros-logs` contains the robot applications output. The `ros-bags` will contain recordings of all events that can be used as simulation input for other tasks. For the simulation or robot application download one of the files and open locally. This will give you an idea of what types of events are saved for review or future use.

18. S3 contains the log files of the output of the applications but you can also get those from CloudWatch Logs.  Navigate to the CloudWatch console and select Logs from the left. In here you will see a Log Group names `/aws/robomaker/SimulationJobs`. Click on that, and then click on your simulations ID for either RobotApplicationLogs or SimulationApplicationLogs and view the entries.

## Activity wrap-up

In this activity, you have worked two of the main AWS RoboMaker products: Development environment and Simulation jobs. The Development environment provides a web-based cloud IDE with all main ROS packages and dependencies built in. You can create as many as needed, and when not in use, the IDE will automatically suspend itself to reduces costs. Here you can work to create, test and build/bundle your code.

When ready to test, the creation of Simulation job automates the process of combing the two applications, provides a web-based graphical interaction with your robot and the simulated world, and logs the output of all components in multiple manners.

Some of the key benefits of using AWS RoboMaker include:

* Not limited to local resources - By using the cloud, AWS RoboMaker provides resources as needed and only for when you need them.
* Develop anywhere on any web-based device - By streaming the output of graphically intensive applications such as Gazebo instead of processing it locally, you can use a desktop, laptop, or even tablet to develop and interact with simulations.
* Scale - This activity showed a single simulation of an application. If the need to test a robot against different environments (simulations) is needed, or testing different iterations of robot applications against a single simulated environment, both of these can be done by scaling to multiple simulation jobs.

### Clean-up Reminder

In this activity, you created a Development environment, CloudWatch logs, and S3 objects that incure cost. If you **are not continuing** on with the next sections of the workshop, remember to go to the [clean-up steps](https://www.robomakerworkshops.com/workshop/cleanup/) and remove these resources to stop any potential costs for occurring.