---
title: "Prep: Workshop Setup"
chapter: true
weight: 3
---

# Workshop Setup 

To complete this workshop you need an **AWS account with administrative permissions** as this is needed to create or modify resources and allow AWS RoboMaker to interact with services on your behalf. 

Please follow these steps to setup your account:

1. [Log in via the AWS console](https://console.aws.amazon.com/) and take note of the region you are using. 

2. If you are using your own account for this exercise and have a credit code, apply it now. To apply the credit code, select your username from the twop right corner of the AWS console and click **My Account**. Next, click on **Credits**.
   
      > **Important:** *The credit codes provided will cover the cost of this workshop. However, you must clean-up the resources after the workshop has completed.*

3. Create a **cheat sheet** text file to hold values needed for the activities. When you see "*store for later use*", this is where the details should go and be referenced.

4. Create an S3 bucket and make note of the bucket name. This bucket will hold all of the AWS RoboMaker related objects. 

    We will reference `<YOUR_S3_BUCKET_NAME>` throughout the workshop. For example:  `s3://<YOUR_S3_BUCKET_NAME>/logfiles`. s3 bucket names need to be unique, but short.

5. Open Kinesis Video Streams and create a new stream with the following configuration. When creating the stream, **uncheck "Use default settings"**):

    * **Stream Name**: *roboMaker_TurtleBot3*
    * **Data Retention Period**: *1 hour*

    *Note the Stream name (`roboMaker_TurtleBot3`) for later use.*

    ![kinesis-streams](../../images/kinesis-streams.png)

6. Finally, we will create the default resources for the lab. Click on the launch stack button below.

    [![Launch Stack](https://cdn.rawgit.com/buildkite/cloudformation-launch-stack-button-svg/master/launch-stack.svg)](https://console.aws.amazon.com/cloudformation/home#/stacks/new?stackName=buildkite&templateURL=s3://assets.robomakerworkshops.com/cfn/bootstrap.cfn.yaml)

    This will create a **VPC** and pair of **subnets** to run AWS RoboMaker instances in. Once the stack has launched, take note of the **outputs**. We will use these values throughout the workshop. 

**Congratulations!** You have compelted the setup process of the workshop. 
