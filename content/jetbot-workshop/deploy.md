---
title: "Activity #4: Deploying with RoboMaker"
chapter: true
weight: 4
description: In this activity you will explore how to configure a robot and a fleet, and deploy a software bundle to your physical robot.
---

# Deploying with RoboMaker
When a robot application is deployed to a physical robot, AWS RoboMaker does the following:

- AWS RoboMaker creates or updates a custom Lambda in your account. The Lambda contains the logic needed for deployment. This includes robot application bundle download, ROS launch, pre- and post-checks, and other logic.

- AWS RoboMaker begins deployment to the fleet using the parallelization specified in the deployment configuration.

- AWS RoboMaker notifies AWS IoT Greengrass to run the custom Lambda on the target robot. The daemon running on the robot receives the command and runs the Lambda. If a Lambda is running when the command is received, it and all ROS process on the robot are terminated.

- The Lambda downloads and uncompresses the robot application bundle from Amazon S3. If a pre-launch script was specified, it is run. Then the Lambda starts ROS using the launch file and package specified. If a post-launch script was specified, it is run after ROS is started. Finally, the status of the deployment is updated.

### Create a Robot Application
1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left pane, choose Development, and then choose Robot applications.

1. Select **Create robot application**.

1. In the Create robot application page, type a Name for the robot application. Choose a name that helps you identify the robot.

1. Select the Robot software suite used by your robot application
    * Select *ROS Melodic*

1. Provide the Amazon S3 path to your bundled robot application file in **ARM64 source file** field. If this robot application is used only in simulations, specify a bundle built for the ARM64 platform. If you use this robot application in a fleet deployment, specify one or more bundles that represent the architectures of the robots in your fleet.

1. Choose Create.


#### Create a Robot Application Version
1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose Development, and then choose Robot applications.

1. Choose the robot application name.

1. In the Robot applications details page, choose Create new version, and then choose Create.

### Create a Robot

To create a robot:

1. Open the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose Fleet Management, and then choose Robots.

1. Choose Create robot.

1. In the Create robot page, type a name for the robot.

1. Select the Architecture of the robot.
  1. Select the ARM64 architecture for the Sparkfun Robot

1. Under AWS IoT Greengrass group defaults, select a Create new to create a new AWS IoT Greengrass group for the robot. 
    *Optionally, you can select an existing AWS IoT Greengrass group. Each robot must have its own AWS IoT Greengrass group.*

    1. If you use an existing AWS IoT Greengrass group, it must have an IAM role associated with it. To create the role, see Create deployment role.

1. Select a IAM role to assign to the AWS IoT Greengrass group created for the robot. It grants permissions for AWS IoT Greengrass to access your robot application in Amazon S3 and read update status from AWS RoboMaker.

1. Choose Create.

1. In the **Download your Core device** page, choose **Download** to download and store your robot's security resources.


### Configure Robot with Certificates
AWS RoboMaker uses X.509 certificates, managed subscriptions, AWS IoT policies, and IAM policies & roles to secure the applications that run on robots in your deployment environment.

An AWS RoboMake robot is also a Greengrass core. Core devices use certificates and policies to securely connect to AWS IoT. The certificates and policies also allow AWS IoT Greengrass to deploy configuration information, Lambda functions, connectors, and managed subscriptions to core devices

1. On your local machine, open a terminal and navigate to the location of the dowloaded security resources from the previous step.

1. Locate the IP address of robot on the OLED
![Sparkfun Jetbot OLED display](https://cdn.shopify.com/s/files/1/0915/1182/products/14532-SparkFun_Micro_OLED_Breakout__Qwiic_-01_300x.jpg)

1. Unzip your device certificates to the robot:

    ```
    # Copy the local security resources to the robot
    $ scp /path/to/downladed-zip/<robot-certs>.zip jetbot@<ip-addres>:/home/jetbot/robomaker-robot-certs.zip

    # SSH to the robot
    $ ssh jetbot@<ip-address>

    # Switch to the root user
    $ sudo su -s /bin/bash

    # Unzip the jetbot security credentials to greengrass certificate store
    $ unzip /home/jetbot/<greengrass-certs>.zip -d /greengrass
    
    # update the CA certificate used by RoboMaker
    $ cd /greengrass/certs/
    $ wget -O root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem
    
    # start greengrass core
    $ sudo /greengrass/ggc/core/greengrassd start
    
    # Exit the root shell
    $ exit # or Ctrl-d

    # Terminate the ssh connection
    $ exit # or Ctrl-d
    ```

### Create a Fleet
1. Sign in to the AWS RoboMaker

1. In the left navigation pane, under **Fleet Management**, and then choose **Fleets**.

1. Select Create fleet.

    - In the Create fleet page, type a name for the fleet.


1. Click Create to create the fleet.


#### Register a Robot

1. In the left navigation pane, choose Fleet Management, and then choose Fleets.

1. Select the Name of the fleet you want to modify.

1. In the Fleet details page, select Register.

1. In the Register robots page, select the robot you want to register, then select Register robots.


### Create a Deployment
1. Sign in to the AWS RoboMaker console at https://console.aws.amazon.com/robomaker/

1. In the left navigation pane, choose Fleet Management, and then choose Deployments.

1. Click Create deployment.

1. In the Create deployment page, under Configuration, select a Fleet.

1. Select a Robot application.

1. Select the Robot application version to deploy. The robot application must have a numbered `applicationVersion` for consistency reasons. If there are no versions listed, or to create a new version, see Creating a Robot Application Version.

1. Under Deployment launch config, specify the Package name: `jetbot`

1. Specify the Launch file: `rc.launch`
  

1. Environment variables, type in an environment Name and Value. Environment variable names must start with A-Z or underscore and consist of A-Z, 0-9 and underscore. Names beginning with “AWS” are reserved.

    - Add the following environment variables:
        - **Key** = `MOTOR_CONTROLLER`(key must be in all caps exactly as MOTOR_CONTROLLER) **Value** = `qwiic`
                      
1. Specify a Robot deployment timeout. Deployment to an individual robot will stop if it does not complete before the amount of time specified.

1. Click Create to create the deployment job.

------

Keep track of the progress of the deployment, when copying and extracting completes, the status will change to **Launching**.  

**Congratulations, you can now control your robot with the virtual joystick!**




