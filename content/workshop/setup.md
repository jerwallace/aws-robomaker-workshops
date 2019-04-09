---
title: "Prep: Workshop Setup"
chapter: true
weight: 3
---

# Workshop Setup 

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

   ![kinesis-streams](../../images/kinesis-streams.png)

6. Ensure the default VPC is available for use and has Internet access (either public IP address or NAT gateway). **Make sure you are looking at Default VPC in us-west-2!**

7. Finally, we will reference **subnet IDs** and **security group IDs** later on in the workshop. We can capture these from the automatic dropdowns when creating a new simulation. In the next few steps, we will start to create a simulation for the sole purpose of capturing these IDs. **Therefore, do not actually create the simulation, you won't start one yet.**  We'll do that later.  To get the information about your VPC network, do the following:

   1. *RoboMaker->Simulation jobs->Create simulation job*

   2. Ignore the first two blocks, and in the Networking block, select the *VPC* you would like to use

   3. This will populate the available *Subnets* and *Security Groups*

   4. Under Subnets select two or more from the drop-down, then you can drag-select with your mouse the identifiers for them and place them in your cheat sheet for further use.

   5. Do the same for the *Security Groups* section (only select a single Security Group).

   6. Copy the subnet IDs and security group IDs referenced here for use later in the workshop:

      ![1_select_subnets](../../images/1_select_subnets.png)
      
   
   8. Click **Cancel**.  **Do not create this simulation job at this time.**

**Congratulations!** You have compelted the setup process of the workshop. 
