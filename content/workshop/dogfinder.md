---
title: "Activity #2: Find Fido, Dog Finder Robot"
chapter: true
weight: 8
description: In this activity you will explore other methods for building and deploying applications in the development environment, and see how an application can directly interact with AWS services in a ROS native manner (nodes and topics), and how any AWS service can be used through normal SDK calls (python boto3 in this instance).
---

# Cloud service integration to do object detection

![2_dog](../../images/2_dog.jpg)

Our objective: Find Fido!

This activity covers working with a robot application that integrates with other AWS services. The robot will work in virtual world and turn and detect images, looking for one includes a dog.

When complete, you will have learned:

* Using commands in the terminal to build and bundle our applications
* Submitting a simulation job programmatically from the command line
* Reviewing the output of Gazebo against CloudWatch logs posts directly from the robot
* Show how robot camera output can be sent to Kinesis Video Streams for further use
* Use `rostopic` commands to send a message to the robot to restart it's goal seeking once the dog image has been found

## Activity tasks


1. For this activity, you will be using **three terminal tabs** to work side-by-side on the simulation and robot application directories, while using the third tab for working with the operating system.

    Close all terminal windows (bash, Immediate, etc.)  and then use the green plus sign to open three tabs like this:

    ![2_tabs](../../images/2_tabs.png)

    When a task says to "From the **SIM TAB** run XXX", use the second/middle tab named "sim".

2. The project we'll be working with is located in GitHub.  You need to clone it into the Cloud9 environment so you can work with it.  From the **OS TAB**, run the following commands to clone the repository:

    ```bash
    cd ~/environment

    # clone the DogFinder repository
    git clone https://github.com/jerwallace/aws-robomaker-sample-application-dogfinder.git
    ```

3. To build the robot application, issue the following commands from the **ROBOT TAB**:

    ```bash
    cd aws-robomaker-sample-application-dogfinder/robot_ws/

    # Ensure latest packages
    sudo apt-get update

    # Pull in ROS packages (errors seen early on can be ignored)
    # This 5-10 minutes to complete
    rosdep install --from-paths src --ignore-src -r -y

    # Build the robot application
    colcon build
    ```

4. Once that is complete, build the simulation application from the **SIM TAB**:

    ```bash
    cd aws-robomaker-sample-application-dogfinder/simulation_ws/

    # rosdep again - will complete quickly
    rosdep install --from-paths src --ignore-src -r -y

    # Build the simulation application - will complete quickly
    colcon build
    ```

    The initial ROS dependency and build process takes a longer time due to all the external packages that need to be downloaded, compiled, and/or installed. As you make small changes to code and iterate, the build process becomes much faster. The initial build time is a good reason to size your Cloud9 IDE accordingly.

    At this point both robot and simulation application are ready. The simulation application will have the hexagon world ready with the TurtleBot3 centered, and the robot application has been built with native integration to CloudWatch Logs, Metric and Kinesis Video Streams; and boto3 support to send images to Amazon Rekognition for object detection.

    However, since we cannot simulate from the Cloud9 IDE,continue to bundle both applications.

5. To bundle the robot application, from the **ROBOT TAB** execute the following:

    ```bash
    #make sure colcon bundle is the latest version.  This only needs to be run once in the Cloud9 environment
    sudo pip3 install -U colcon-bundle colcon-ros-bundle

    #create the bundle for the robot application
    colcon bundle
    ```

    Once successfully completed, do the same on the **SIM TAB**:

    ```bash
    #create the bundle for the simulation application
    colcon bundle
    ```

    These two operations will create complete tar files for use and will write them to eaach workspaces' respective bundle directory.

    *Why do I have to go through all these steps when in the previous activity I just clicked a menu command and magic happened?!?!*

    That's one of the benefits of AWS RoboMaker, the ability to wrap the complexity of ROS into a few commands. In the background all of the same steps were being taken as you just completed. By doing this step-by-step,  you can see process is to build and deploy a robot application. In a lot of situations you will have to go through similar setups for your applications, so having familiarity with it is helpful.

6. With both applications built, you will now copy them to S3 so they can be used by the simulation service. For both applications, copy to S3:

    From the **ROBOT TAB**:

    ```bash
    # Replace <YOUR_BUCKET_NAME> with your bucket
    aws s3 cp bundle/output.tar s3://<YOUR_BUCKET_NAME>/dogfinder/output-robot.tar
    ```

    and from the **SIM TAB**:

    ```bash
    # Replace <YOUR_BUCKET_NAME> with
    aws s3 cp bundle/output.tar s3://<YOUR_BUCKET_NAME>/dogfinder/output-sim.tar
    ```

7. With the bundle files ready, create a simulation job from the OS TAB. In the root of the DogFinder directory is a file named `submit_job.sh`. Double-click it and replace the entries at the top of the file with your specific ones (S3 bucket, VPC details, etc.), **and then save**. There is a complete one in your **CloudFormation > Outputs**. It should look similar to this:

    ```bash
     #!/bin/bash
     # Example - replace with your own
     export BUCKET_NAME="<YOUR_BUCKET_NAME>"
     export SUBNETS="subnet-e2xxx795,subnet-e2xxx123"
     export SECURITY_GROUP="sg-fe2xxx9a"
     export ROLE_ARN="arn:aws:iam::1234565789012:role/robomaker_role"
    ```

8. In the **OS TAB**, run the script which will create the robot and simulation applications, then create and start the simulation job:

    ```bash
    # script in top-level of DogFinder/ directory, adjust as needed
    aws-robomaker-sample-application-dogfinder/submit_job.sh
    ```

    A successful launch will return a JSON document with all the details including an *arn* with the simulation job value:

    ```json
    "arn": "arn:aws:robomaker:us-west-2:123456789012:simulation-job/sim-8rcvbm7p023f",
    ```

9. At this point you can open a AWS RoboMaker console and check the status of the simulation job. It will take a few minutes to go from *Pending* to *Running*, but that point you can open Gazebo and Terminal applications.

     Notice in Gazebo as you pan around that the robot if facing north at the picture of the bridge. Right now the robot is waiting for a message to start goal seeking and finding the picture of the dog. Before you issue the command from the simulation terminal, let's bring up the following windows and resize so we can see them all (the may take a bit of adjusting):

     * *Kinesis Video Streams* console, then click on your stream
     * *Gazebo*, zoom in to the hexagon and the robot
     * *Simulation job Terminal*, where you will issue the start command

     ![2_all_windows](../../images/2_all_windows.png)

     You don't need to see too much of the video stream window in the background, just enough to see it steaming video.

10. At this point, in Gazebo the robot should be facing upwards (due North); the video stream should show the bridge photo; and CloudWatch logs should show a message "Waiting to start finding Fido". Now from the terminal, you will send a message to a topic the robot is listening on to start the goal seeking action:

     ```bash
     rostopic pub --once /df_action std_msgs/String 'start'
     ```

     What this will do is publish (`pub`) a single message (`--once`) to the topic your robot is listening on (`/df_action`), and will send a string type  (`std_msgs/String`) with the command to process (`start`). The robot will receive this command and start the task (turn and process images), looking for our target, a picture of a dog.

     *When you see the robot start turn in Gazebo, if the video stream doesn't update, click the "fast-forward" button to forward to realtime.*

     Once a dog has been found, the robot will stop and log in CloudWatch Logs->Log Groups->dogfinder_workshop->TurtleBot3 informational messages on finding the dog!

     ![2_dog_logs](../../images/2_dog_logs.png)

11. Once the dog image is found, the robot waits for the next command to start the process again. You can issue the `rostopic pub` command again in the terminal to start the process again.

12. At this point if there is time, feel free to investigate the other applications and look at how the code is working. For example, if you'd like to see the robot's view of the world via rqt, open the rqt application and from the rqt menu select Plugins->Visualization->Image View and then in Image View drop down, select /camera/rgb/image_raw.

## Activity wrap-up

In this activity, you built and simulated a robot application that not only interacts (turns) in Gazebo, but also utilizes other AWS services. This include ROS native integration such as CloudWatch logs and Kinesis Video Streams, or through the flexible use of AWS SDK's such as boto3 to interact with other AWS services such as Amazon Rekognition.

What was covered:

* Working with ROS applications inside the Development Environment at a command line level
* Understanding the resources required to compile (build) and package (bundle) a ROS application
* Using the AWS CLI to interact with AWS RoboMaker to create applications, upload bundles, and launch simulation jobs
* Using AWS services such as Kinesis Video Streams and CloudWatch logs directly from ROS to interact with the robot (virtual or real)
* Use standard SDK's to interact with other AWS services

### Clean-up

In this activity, you created a Development environment, CloudWatch logs, and S3 objects that incure cost. Please follow the clean-up steps in the main. README document on how to remove these and stop any potential costs for occurring.
