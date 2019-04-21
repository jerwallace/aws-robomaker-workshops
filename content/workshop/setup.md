---
title: "Prep: Workshop Setup"
chapter: true
weight: 3
---

# Workshop Setup 

### Log Into the AWS Console

To complete this workshop you need an **AWS account with administrative permissions**. This is needed to create or modify resources and allow AWS RoboMaker to interact with services on your behalf. 

If you are in a classroom setting, for this exercise we will provide a credit code, which you should apply it now. To apply the credit code, select your username from the top right corner of the AWS console and click **My Account**. Next, click on **Credits**. 
      
   **Important:** *The credit codes provided will cover the cost of this workshop. However, you must clean-up the resources after the workshop has completed.*

### Launch Cloudformation Stack

Once you have successfully signed into the AWS console, launch the following cloudformation stack to create the required resources:

[![Launch Stack](../../images/launch-stack.svg)](https://console.aws.amazon.com/cloudformation/home#/stacks/new?stackName=robomaker-wrk&templateURL=https://s3.amazonaws.com/assets.robomakerworkshops.com/cfn/bootstrap.cfn.yaml)

This will create: 

   - a **VPC** and pair of **subnets** and a **default security group** to run AWS RoboMaker instances in. 
   - an **S3 bucket** to store your Robomaker assets (such as application bundles).
   - **Two IAM roles** that you will use for the workshop.

Once the stack has launched, take note of the **outputs**. We will use these values throughout the workshop. 

### Create Kinesis Video

Finally, open the console for [Kinesis Video Streams](https://console.aws.amazon.com/kinesisvideo/home) and create a new stream with the following configuration. When creating the stream, **uncheck "Use default settings"**:

   - **Stream Name**: *roboMaker_TurtleBot3*
   - **Data Retention Period**: *1 hour*

![kinesis-streams](../../images/kinesis-streams.png)    
*Note the Stream name (`roboMaker_TurtleBot3`) for later use.*

**Congratulations!** You have completed the setup process of the workshop. 
