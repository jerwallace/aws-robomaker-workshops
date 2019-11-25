---
title: "Prep: Workshop Setup"
chapter: true
weight: 1
description: "We will start by setting up your AWS account to develop robot applications with AWS RoboMaker."
---

# Workshop Setup 

### Log Into the AWS Console

To complete this workshop you need an **AWS account with administrative permissions**. This is needed to create or modify resources and allow AWS RoboMaker to interact with services on your behalf.

**Note: We will use us-west-2 (Oregon) for this workshop.**

If you are in a classroom setting, we will provide a credit code which you should apply it now. To apply the credit code, select your username from the top right corner of the AWS console and click **My Account**. Next, click on **Credits**. 
      
   **Important:** *The credit codes provided will cover the cost of this workshop. However, you must clean-up the resources after the workshop has completed.*

### Launch Cloudformation Stack

Once you have successfully signed into the AWS console, launch the following cloudformation stack to create the required resources:

[![Launch Stack](../../images/launch-stack.svg)](https://console.aws.amazon.com/cloudformation/home#/stacks/new?templateURL=https://s3.amazonaws.com/assets.robomakerworkshops.com/cfn/bootstrap.cfn.yaml&region=us-west-2)

This will create: 

   - a **VPC** and pair of **subnets** and a **default security group** to run AWS RoboMaker instances in. 
   - an **S3 bucket** to store your Robomaker assets (such as application bundles).
   - **Two IAM roles** that you will use for the workshop.

Once the stack has launched, take note of the **outputs**. We will use these values throughout the workshop. 

### Create Teleop User Using Cloudformation

[![Launch Stack](../../images/launch-stack.svg)](https://console.aws.amazon.com/cloudformation/home#/stacks/new?templateURL=https://s3.amazonaws.com/assets.robomakerworkshops.com/cfn/teleop_creds_creation.cfn.yaml&region=us-west-2)

This will create:

   - **One IAM role** that will allow you to teleop your robot.  Make sure to save the credentials created (Access Key & Secret) at this part of the workshop!

**Congratulations!** You have completed the setup process of the workshop. 
