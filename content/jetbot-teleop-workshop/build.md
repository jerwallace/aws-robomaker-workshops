---
title: "Activity #2: Deploying ROS Applications"
chapter: true
weight: 3
requiredTimeMins: 15
description: In this activity you will explore other methods for building and deploying applications in the development environment including building for an ARM64 device using Docker.
---

# Deploying ROS Applications 
In the next activity, you will deploy the JetBot application code to a physical robot. As your workshop instructor mentioned, we will be working in groups for this activity. Get together with your group and pick up an NVidia JetBot. When a robot application is deployed to a physical robot, AWS RoboMaker does the following:

- AWS RoboMaker creates or updates a custom Lambda in your account. The Lambda contains the logic needed for deployment. This includes robot application bundle download, ROS launch, pre- and post-checks, and other logic.
- AWS RoboMaker begins deployment to the fleet using the parallelization specified in the deployment configuration.
- AWS RoboMaker notifies AWS IoT Greengrass to run the custom Lambda on the target robot. The daemon running on the robot receives the command and runs the Lambda. If a Lambda is running when the command is received, it and all ROS process on the robot are terminated.
- The Lambda downloads and uncompresses the robot application bundle from Amazon S3. If a pre-launch script was specified, it is run. Then the Lambda starts ROS using the launch file and package specified. If a post-launch script was specified, it is run after ROS is started. Finally, the status of the deployment is updated.

### Prepare Your ROS Application for Deployment

The NVidia Jetson Nano Developer Kit has an **arm64** architecture. Therefore, to prepare the application to deploy and launch on our JetBot, we need to build and bundle the ROS Application for arm64. To do this, we will use a docker image we prepared to **cross compile** the application.

1. Open the RoboMaker IDE and navigate to the terminal

1. **IMPORTANT**: Change to the **jetbot** directory and build & bundle the ROS application in a docker container.
    ```
    # Make sure you are in the jetbot directory
    $ cd ~/environment/jetbot
    ```

1. Next, start a docker container with the following command. This will enable you to run commands inside the docker container once running. *Note: if you see a permission denied error in container shell, it is safe to ignore and continue.*

    ```
    # IMPORTANT: Make sure you are in the jetbot directory
    # Build and bundle the robot application
    $ docker run --rm -ti -v $(pwd):/environment/jetbot jetbot-ros
    ```

1. To cross compile the application for arm64, run the script `compile_arm64.sh`. 

    ```
    # You will be dropped into the shell of the docker container
    # If you see a permission denied error, it is safe to ignore and continue

    (docker)$ ./assets/scripts/compile_arm64.sh 

    # Wait until shell script is completed 
    #Type exit or Ctrl-D, to exit the container
    (docker)$ exit
    ```

    - **What is happening here?** The compile_arm64.sh script simply runs a ros dependency (rosdep) update and install to collect and build any dependencies. Next, it runs the build commands `colcon` and `colcon bundle`. If you want, open it up and check it out.

1. Open *roboMakerSettings.json* file located in the root folder under RoboMaker IDE folder pane. Look for **s3Bucket** and write down the name of the bucket. It will look similar to *mod-47118164636e49dc-s3bucket-1t252d6l3fmil*. You will need this for next step to upload the compiled and bundled ARM64 ROS application. Also find and write down the value in IOT_ENDPOINT, you will need it when deploying the application to the JetBot.

![S3 and IoT Endpoint](../../images/s3-iot-endpoint.png)

1. Back in the RoboMaker IDE and navigate to the terminal. *Note: Make sure you exited out of the container in previous step.*
    ```
    # Make sure you exited out of the container in previous step
    # Copy the robot application to S3
    $ aws s3 cp ./robot_ws/arm64_bundle/output.tar s3://<S3-BUCKET-NAME>/jetbot/aarch64/output.tar
    ```

Congratulations! Now, you should have an arm64 bundle (output.tar) of your ROS application ready to deploy.  Next, we will configure our Robot Application and Robot in AWS RoboMaker.

### Create a Robot Application

1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left pane, choose **Development**, and then choose **Robot Applications**.

1. Select **Create Robot Application**.

1. In the *Create Robot Application* page, type in a **name** for the robot application. Choose a name that helps you identify the robot application.

1. Select the Robot software suite used by your robot application
    * Select *ROS Melodic*

1. Provide the Amazon S3 path to your bundled robot application file in **ARM64 source file** field. Click **Browse S3** and locate the tar file that you uploaded in previous step, `s3://S3-BUCKET-NAME/jetbot/aarch64/output.tar`. Click Create.

1. Open the **Robot Application** you just created. This will open the application details page. Choose **Create New Version**, and then choose **Create**.

### Create a Robot

1. Return to the RoboMaker Console: https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose **Fleet Management**, and then choose **Robots**.

1. Choose **Create Robot**.

1. In the **Create Robot** page, type a name for the robot.

1. Select the *Architecture* of the robot. Select **ARM64 architecture**, which is the architecture of the JetBot.

1. Under **AWS IoT Greengrass** group defaults, select *Create New* to create a new AWS IoT Greengrass group for the robot.

    *Optionally, you can select an existing AWS IoT Greengrass group. Each robot must have its own AWS IoT Greengrass group.*

    1. If you use an existing AWS IoT Greengrass group, it must have an IAM role associated with it. To create the role, see Create deployment role.

1. Select the *existing IAM role* to assign to the AWS IoT Greengrass group created for the robot. It grants permissions for AWS IoT Greengrass to access your robot application in Amazon S3 and read update status from AWS RoboMaker.

1. Choose **Create**.

1. In the **Download Your Core Device** page, choose **Download** to download the zip file which includes your robot's security resources.

### Configure Robot with Certificates

AWS RoboMaker uses X.509 certificates, managed subscriptions, AWS IoT policies, and IAM policies & roles to secure the applications that run on robots in your deployment environment.

An AWS RoboMake robot is also a Greengrass core. Core devices use certificates and policies to securely connect to AWS IoT. The certificates and policies also allow AWS IoT Greengrass to deploy configuration information, Lambda functions, connectors, and managed subscriptions to core devices

1. On your local machine, open a terminal and navigate to the location of the dowloaded security resources from the previous step.

1. Locate the IP address of robot on the OLED
![waveshare oled](../../images/waveshare-oled.png)

1. Open the local Jypter server by typing this into your web browswer: `https://<IP_ADDRESS>:8888`.

1. In this browser, you will be able to easily acccess the filesystem on the **JetBot**. Select **Upload** to upload your downloaded keys to the JetBot. 

1. Next, open a **terminal** and copy the keys to your greengrass folder. The password for su is "**jetbot**".

    ```
    # Switch to the root user (password is "jetbot")
    $ sudo su

    # Unzip the jetbot security credentials to greengrass certificate store
    $ unzip /home/jetbot/<greengrass-certs>.zip -d /greengrass
    
    # update the CA certificate used by RoboMaker
    $ cd /greengrass/certs/
    $ wget -O root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem
    
    # start greengrass core
    $ sudo /greengrass/ggc/core/greengrassd start
    ```

### Create a Fleet

1. Return to the RoboMaker Console: https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, under **Fleet Management**, and then choose **Fleets**.

1. Select **Create Fleet**.

    - In the *Create Fleet* page, type a name for the fleet.

1. Click **Create** to create the fleet.

#### Register a Robot

1. In the left navigation pane, choose Fleet Management, and then choose Fleets.

1. Select the Name of the fleet you want to modify.

1. In the Fleet details page and under Registered robots section, select Register new.

1. In the Register robots page, select the robot you want to register, then select Register robots.

### Create a Deployment

1. Return to the RoboMaker Console: https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose **Fleet Management**, and then choose **Deployments**.

1. Click **Create Deployment**.

1. In the **Create Deployment** page, under **Configuration**, select a **Fleet**.

1. Select a **Robot Application**.

1. Select the **Robot Application** version to deploy. The robot application must have a numbered `applicationVersion` for consistency reasons. If there are no versions listed, or to create a new version, see Creating a Robot Application Version.

1. Under **Deployment Launch Config**, specify the Package name: `jetbot_app`

1. Specify the Launch file: `teleop.launch`
  
1. Environment variables, type in an environment Name and Value. Environment variable names must start with A-Z or underscore and consist of A-Z, 0-9 and underscore. Names beginning with “AWS” are reserved.

    - Add the following environment variables:
        - **Key** = `IOT_ENDPOINT`(key must be in all caps exactly) **Value** = `<your IoT endpointAddress>` (this is the IOT_ENDPOINT you captured from earlier step in roboMakerSettings.json file)
        - **Key** = `ROBOT_NAME`(key must be in all caps exactly) **Value** = `joystick1`(do not change the value)
                      
1. Specify a Robot deployment timeout. Deployment to an individual robot will stop if it does not complete before the amount of time specified.

![JetBot Deployment](../../images/jetbot-deployment.png)

1. Click Create to create the deployment job.

1. Keep track of the progress of the deployment, when copying and extracting completes, the status will change to **Launching**.  

### Drive your JetBot!

1. Open the **robogui.html** file in a browser and make sure the connection status states: *Connected*.

1. Use your mouse to move the joy stick and drive the Jetbot in real world!

Congratulations!!! You have completed the workshop! 

### Clean-up Reminder

**Note for those running in their own AWS accounts and are not using a workshop code:**

In this activity, you created a some resources (Development environment, CloudWatch logs, and S3 objects) that incure cost. If you **are not continuing** on with the next sections of the workshop, remember to go to the [clean-up steps](https://www.robomakerworkshops.com/workshop/cleanup/) and remove these resources to stop any potential costs for occurring.