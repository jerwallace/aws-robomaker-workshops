---
title: "Activity #2: Deploying an ML-enabled ROS Application"
chapter: true
weight: 3
requiredTimeMins: 15
description: In this activity you will explore other methods for building and deploying ML-enabled applications in the development environment including building for an ARM64 device using Docker. You will also package up 2 machine learning models and deploy them with your application.
---

# Deploying ROS Applications 
In the next activity, you will deploy the JetBot application code to a physical robot along with two ML models. As your workshop instructor mentioned, we will be working in groups for this activity. Get together with your group and pick up an NVidia JetBot. 

For this activity, you will need to connect to our provisioned WiFi. Here it is:
    ```
    SSID: Condensate
    PW: !Dev1ces+Are+Com1ng!
    ```
    
**About the Application**

The application we will be building today highlights how you can deploy machine learning models for inference at the edge with an [NVidia Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit). Our demo application will autonomously navigate in a LEGO world. Machine learning resources represent cloud-trained inference models that are deployed to an AWS IoT Greengrass core.

- The **first** machine learning model has been trained to detect edges in the road and will enable the robot to autonomously navigate on a road tile. 
- The **second** machine learning model has been trained with a small number of labeled dinosaur pictures to detect dinosaurs. 

![App Architecture](../../images/dino-arch.png)

When you use **AWS RoboMaker** to deploy a robot application, the following happens:

- AWS RoboMaker creates or updates a custom Lambda in your account. The Lambda contains the logic needed for deployment. This includes robot application bundle download URL and permissions, ROS launch, pre- and post-checks, and other logic.
- AWS RoboMaker begins deployment to the fleet using the parallelization specified in the deployment configuration.
- AWS RoboMaker notifies AWS IoT Greengrass to run the custom Lambda on the target robot. The daemon running on the robot receives the command and runs the Lambda. If a Lambda is running when the command is received, it and all ROS process on the robot are terminated.
- The Lambda downloads and uncompresses the robot application bundle from Amazon S3. If a pre-launch script was specified, it is run. Then the Lambda starts ROS using the launch file and package specified. If a post-launch script was specified, it is run after ROS is started. Finally, the status of the deployment is updated.

### Collect Your JetBot Application for Deployment. 

The NVidia Jetson Nano Developer Kit has an **arm64** architecture. Therefore, to prepare the application to deploy and launch on our JetBot, you would need to build and bundle the ROS Application for arm64. Also, since we are using PyTorch and other local dependencies to run inference at the edge, you would need to bundle these dependencies with your application. We have included a docker file to build and bundle our simple road following application. However, since this is a long process - we have already done this for you. 

Therefore, in this next step, we will copy the application bundle to an S3 bucket in your account.
    
 1.  Get your S3 bucket name from the CloudFormation template in the CloudFormation console (from the previous section) by going [here](https://console.aws.amazon.com/cloudformation/).  It will have the format of "mod-*********-s3Bucket-********".

1. Navigate to the terminal in the RoboMaker IDE.
    ```
    # Make sure you exited out of the container in previous step
    # Copy the robot application to S3
    $ aws s3 cp s3://jetbot-workshop-us-east-1-rmw-assets/jetbot/robot_ws/output.tar s3://<S3-BUCKET-NAME>/jetbot/aarch64/output.tar
    ```

Congratulations! Now, you should have an arm64 bundle (output.tar) of your ROS application ready to deploy.  Next, we will configure our Robot Application and Robot in AWS RoboMaker.

### Stage the two ML models in S3

**Note: Wait to move on to this step until the latest models have been created. Your workshop instructor will let you know!**

For today's workshop, we have been activly training a new road following ML model for you to use. However, if you are interested, you can checkout the python notebooks [here](https://github.com/waveshare/jetbot/tree/master/notebooks). We will deploy this model with our **Road Following** application. 

    ```
    # Make sure you exited out of the container in previous step
    # Copy the robot application to S3
    $ aws s3 cp s3://dinobot-models/models.zip s3://<S3-BUCKET-NAME>/models.zip
    ```

Great! Now, we will be able to reference these models with our AWS RoboMaker Deployment.

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

** PLEASE ENSURE ANY VPN YOU USE IS DEACTIVATED.  THE FOLLOWING STEPS WILL NOT WORK IF YOU'RE CONNECTED TO A VPN **

1. Open the local Jupyter server by typing this into your web browser: `http://<IP_ADDRESS>:8888`.

1. In this browser, you will be able to easily access the filesystem on the **JetBot**. Select **Upload** to upload your downloaded keys to the JetBot. 

1. Next, open a **terminal** and copy the keys to your greengrass folder. The password for su is "**jetbot**".

    ```
    # Switch to the root user (password is "jetbot")
    $ sudo su
    
    # Add the ggc_user to the video group so it has access to CUDA drivers
    $ usermod -a -G video ggc_user

    # Unzip the jetbot security credentials to greengrass certificate store
    $ unzip /home/jetbot/<greengrass-certs>.zip -d /greengrass
    
    # update the CA certificate used by RoboMaker
    $ cd /greengrass/certs/
    $ wget -O root.ca.pem https://www.amazontrust.com/repository/AmazonRootCA1.pem
    
    # start greengrass core
    $ /greengrass/ggc/core/greengrassd restart
    ```

### Configure our GreenGrass Lambda Function.

In this section, we will create a simple lambda function application that will copy the ML models to a directory where our ROS application can access them. 
Machine learning resources represent cloud-trained inference models that are deployed to an AWS IoT Greengrass core. To deploy machine learning resources, you will first define a Lambda functions to transfer the model to a location accessible by the ROS application and then add a resources to a Greengrass AWS group.

1. Open the AWS Lambda console

1. Press the **Create Function** button and add the following information:

    * **Function Name**: *choose a function name*
    * **Runtime**: Python 2.7
    Expand the choose or create an execution role
    * Select **Use an existing role** and select the role which is displayed

    ![Create a lambda function view](../../images/lambda_create.png)

1. Choose **Create Function**

1. Open the AWS RoboMaker Cloud9 development environment in another tab and navigate to the [`assets/greengrassModelSync/mlModelSync.py`](https://github.com/jerwallace/assets/greengrassModelSync/mlModelSync.py)

1. Copy the contents of this file and navigate back to the Lambda Function page

1. Paste the contents of the file into the **Function Code** block.

    ![Add lambda function to function code block](../../images/lambda_add_code.png)

1. Click **Save**.

1. Under **Actions**, choose **Publish new version** and enter version **1**

    ![Publish the lambda function](../../images/lambda_publish_version.png)

### Configure Greengrass to Sync ML Models

#### Add a Lambda function to a Greengrass group

1. Open the AWS Console and navigate to **IoT Core**

    ![alt Add a lambda function to the Greengrass group](../../images/greengrass_group.png)

1. From the menu bar on the left side, select **Greengrass** > **Groups**, and click on the group corresponding to your Robot.

1. In the Greengrass Group view, select **Lambdas** from the menu

    ![Add a lambda function to the Greengrass group](../../images/gg_group_add_lambda.png)

1. Click **Add Lambda** and add an existing lambda function.  Select the Lambda function created in the last step

1. Once the Lambda function is added, select the **Edit Configuration** in the upper right corner of the lambda function

1. Under **Lambda Lifecycle**, choose **Make this function long-lived and keep it running indefinitely**

1. Leave all other settings as default values and click and **Update**

#### Add a ML Resource to a Greengrass group

1. From the menu bar on the left side, select **Greengrass** > **Groups**, and click on the group corresponding to your Robot.

1. In the Greengrass Group view, select **Resources** from the menu

1. Under **Resources**, choose **Machine Learning** > **Add a machine learning resource**

    ![Add an ML resource to a Greengrass group](../../images/greengrass_add_ml_resource.png)

1. Use the following values to create the resource:
    * **Resource Name**: *select a resource name*
    * **Model Source**: Choose *Upload a model in S3*
      * Locate the Model in S3 in your <S3-BUCKET-NAME>
    * **Local Path**: */trained_models*
    * **Identify resource owner and set access permissions**: *No OS group*
    * **Lambda function affiliations**: *select your ml model detect* function with **Read and write access**

    ![Add an ML resource to a Greengrass group](../../images/greengrass_add_ml_resource_detail.png)

1. Choose **Save** 

#### Add a Local Resource to a Greengrass group
You can configure Lambda functions to securely access local resources on the host Greengrass core device. Local resources refer to buses and peripherals that are physically present on the host, or file system volumes on the host OS. For more information, including requirements and constraints, see Access Local Resources with Lambda Functions and Connectors.

1. From the menu bar on the left side, select **Greengrass** > **Groups**, and click on the group corresponding to your Robot.

1. In the Greengrass Group view, select **Resources** from the menu

    ![Add a local resource to a Greengrass group](../../images/greengrass_local_resource.png)

1. Under **Resources**, choose **Local** > **Add local resource**

    - **Resource name**: *select a resource name*
    - **Resource type**: *Volume*
    - **Source path**: `/tmp`
    - **Destination path**: `/tmp`
       - *The destination path is the absolute path of the resource in the Lambda namespace.*
    - **Group Owner file access permission**: *Automatically add OS group permissions of the Linux group that owns the resource*
    - **Choose permissions for this Lambda function**: *Read and write acccess*

    ![Add a local resource to a Greengrass group](../../images/greengrass_local_resource_detail.png)

1. Click **Save**

#### Allowing Greengrass to access S3
Greengrass must be granted access to the S3 service.  This access is configured by adding a IAM Policy for the Greengrass service role.

1. From the AWS Management Console, navigate to the Identity and Access Management(IAM) service

1. Select **Roles** from the menu bar on the left side and search for **Greengrass_ServiceRole**.

    ![Add AmazonS3FullAccess policy](../../images/iam_role_summary.png)

1. On the **Summary** view, click **Attach Policies** and search for **AmazonS3FullAccess**.

    ![Add AmazonS3FullAccess policy](../../images/iam_role_policy_add.png)

1. Click the check-box next to the name and select **Attach policy**.

Now the ML models will be synced with the robot application code, you'll deploy a ROS application to make use of the models. The ROS application built and bundled 
in the last module contained the necessary code to make use of this model.  You'll create a new deployment for your robot specifying a different launch file.

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

1. Specify the Launch file: `run.launch`
                      
1. Specify a Robot deployment timeout. Deployment to an individual robot will stop if it does not complete before the amount of time specified.

1. Click Create to create the deployment job.

1. Keep track of the progress of the deployment, when copying and extracting completes, the status will change to **Launching**.

** Please be patient -- launching may take a few minutes while it's loading the inference models into memory **

1. Bring your robot over to one of the road tracks. 

Congratulations!!! You have completed the workshop! 

### Clean-up Reminder

**Note for those running in their own AWS accounts and are not using a workshop code:**

In this activity, you created a some resources (Development environment, CloudWatch logs, and S3 objects) that incure cost. If you **are not continuing** on with the next sections of the workshop, remember to go to the [clean-up steps](https://www.robomakerworkshops.com/workshop/cleanup/) and remove these resources to stop any potential costs for occurring.
