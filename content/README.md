# AWS Robotics Day with RoboMaker

Welcome Robot Builders! 

In this workshop you become familiar with AWS RoboMaker, a service that enables you to easily develop, simulate and deploy intelligent robot applications that integrate with AWS services. This includes AWS machine learning services, monitoring services, and analytics services that enable a robot to stream data, navigate, communicate, comprehend, and learn. Today, you will also get hands on with a physical robot (Robotis TurtleBot 3 Burger) to learn how to manage and deploy robot applications to production robots using AWS RoboMaker.

RoboMaker extends the most widely used open-source robotics software framework **Robot Operating System**, or [ROS](http://www.ros.org/). Therefore, this workshop will include references to ROS concepts and tools. No previous ROS experience is required, however, if you would like to learn more check out the [Learning Resources](learning_resources.md) section for references.

**Here is our agenda for the day:**

1. **Pre-workshop Setup**: We will start by setting up your AWS account to develop robot applications with AWS RoboMaker. 

2. **Hello Robot**: In the first activity, we will run a simple HelloWorld application to introduce you to AWS RoboMaker, ROS and robotics development. 

3. **Find Fido**: In the second activity, we will extend the functionality of our robot and build a robot application that searches for dogs.

3. **Deploy Hello Robot Code**: In the final activity we will deploy the hello world robot application to a **TurtleBot Burger 3**. Once successful, the turtlebot will rotate in place. 

Please read through and complete each activity before starting the next. If something doesn't look correct, ask for assistance as we want to make sure the concept covered are well understood. Also, if you find a bug - submit a pull request!

Excited to build a robot using AWS? Let's get started!

## Workshop Setup 

To complete this workshop you need an **AWS account with administrative permissions** as this is needed to create or modify resources and allow AWS RoboMaker to interact with services on your behalf. 

Please follow these steps to setup your account:

1. [Log in via the AWS console](https://console.aws.amazon.com/) and set your region to **Oregon** from the top menu to right of your username. All activities will reference **Oregon (us-west-2)**. 

   :exclamation: All code is currently set only for us-west-2, using any other regions will not work in this iteration.

2. Apply the credit to your account (*Your username->Account->Credits)*. This amount will cover AWS usage for the workshop, as long as you clean up resources at the end.

3. Create a "cheat sheet" text file to hold values needed for the activities. When you see "store for later use", this is where the details should go and be referenced.

4. Create an S3 bucket and make note of the bucket name. This bucket will hold all of the AWS RoboMaker related objects. **We will reference `<YOUR_S3_BUCKET_NAME>` throughout the workshop.** For example:  `s3://<YOUR_S3_BUCKET_NAME>/logfiles`. s3 bucket names need to be unique, but short.

5. Open Kinesis Video Streams and create a new stream:

   * **Stream name**: *roboMaker_TurtleBot3*
   * Uncheck *Use default settings*
   * **Change Data retention period**: to 1 hour
   * Create stream

   Note the Stream name (`roboMaker_TurtleBot3`) for later use.

   ![kinesis-streams](img/kinesis-streams.png)

6. Ensure the default VPC is available for use and has Internet access (either public IP address or NAT gateway). **Make sure you are looking at Default VPC in us-west-2!**

7. Finally, we will reference **subnet IDs** and **security group IDs** later on in the workshop. We can capture these from the automatic dropdowns when creating a new simulation. In the next few steps, we will start to create a simulation for the sole purpose of capturing these IDs. **Therefore, do not actually create the simulation, you won't start one yet.**  We'll do that later.  To get the information about your VPC network, do the following:

   1. *RoboMaker->Simulation jobs->Create simulation job*

   2. Ignore the first two blocks, and in the Networking block, select the *VPC* you would like to use

   3. This will populate the available *Subnets* and *Security Groups*

   4. Under Subnets select two or more from the drop-down, then you can drag-select with your mouse the identifiers for them and place them in your cheat sheet for further use.

   5. Do the same for the *Security Groups* section (only select a single Security Group).

   6. Copy the subnet IDs and security group IDs referenced here for use later in the workshop:

      ![1_select_subnets](img/1_select_subnets.png)
      
   
   8. Click **Cancel**.  **Do not create this simulation job at this time.**

**Congratulations!** You have compelted the setup process of the workshop. 

## Activities

Below are the activities setup for this workshop. Right-click on each and open in a new tab or window. Once an activity has been completed, you can close out the tab, come back here, and do the same for the next one.

:exclamation: Once all activities are done, please then complete the account cleanup section at the bottom of this page.

### [1 - Hello Robot: Development environment and HelloWorld](./1_dev_hello.md)

In this activity you will setup a development environment and build, bundle, and simulate a "Hello World" application.

### [2 - Find Fido: Cloud integration to enable the robot to find objects](./2_dog_finder.md) 

In this activity you will explore other methods for building and deploying applications in the development environment, and see how an application can directly interact with AWS services in a ROS native manner (nodes and topics), and how any AWS service can be used through normal SDK calls (python boto3 in this instance).

### [3 - Deploy Hello Robot Code: Deploy an application to a physical robot](./3_deployment.md) 

In this activity you will explore how to configure a robot and a fleet, and deploy a software bundle to your robot.


## Workshop cleanup

AWS only charges for consumed resources. Please follow the steps below to shutdown or delete resources so you will not be charged.

1. Delete the S3 bucket by selecting the bucket then clicking *Delete* from above the list of buckets.

2. From the AWS RoboMaker console, make sure there are no simulation jobs in progress. If there are, select them and click *Actions->Cancel*.

3. Also from the AWS RoboMaker Console, from Development->Development environments, click on the environment name, *Edit*, then *Delete* from the AWS Cloud9 console.
4. In the CloudWatch Console, under *Logs*, select each LogGroup (`/aws/robomake/SimulationJobs` and `dogfinder_workshop`) and click *Actions->Delete log group*.
5. In Kinesis Video Streams, delete your stream which will release the stored video content.
6. In IAM, delete the *Cloud9-RoboMakerWorkshop* role, and the *Cloud9-RoboMakerWorkshopDeploymentRole*. 
