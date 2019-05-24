---
title: "Prep: Workshop Setup"
chapter: true
weight: 3
---

# Workshop Setup 

### Log Into the AWS Console and set the AWS Region

To complete this workshop you need an **AWS account with administrative permissions**. This is needed to create or modify resources and allow AWS RoboMaker to interact with services on your behalf.

We will use the US West (Oregon) region for this workshop.  In the region menu item, select *US West (Oregon)*.

![Region selection](../../images/mars-rover/region-selection.jpg)

### Launch CloudFormation Stack

AWS CloudFormation provides a common language to describe and provision infrastructure resources in your cloud environment. CloudFormation allows you to use a simple text file to model and provision the resources needed for the workshop.  For this workshop, we've pre-created a template that simplifies some of the setup.  The infrastructure it creates is needed to run the activities.  It will create networking resources, and it will create roles in your account that give AWS RoboMaker the permissions it needs to run the simulations.

Once you have successfully signed into the AWS console, click the button below to launch a CloudFormation stack to create the required resources:

[![Launch Stack](../../images/launch-stack.svg)](https://console.aws.amazon.com/cloudformation/home#/stacks/new?templateURL=https://s3.amazonaws.com/assets.robomakerworkshops.com/cfn/bootstrap.cfn.yaml&region=us-west-2)

1. On the *Create stack* page, accept the defaults and click **Next**.
2. On the *Specify stack details* page, set *Stack name* to a value that will help you identify this stack, such as "reMARS-robot-workshop-resources".
3. In the *Parameters* section, the networking configuration has been pre-populated.  Leave the default values.
4. For the *s3BucketName* field, the value must be globally-unique, and it must be lower case.  This is because it will be used in the domain name for the S3 bucket that gets created.  For today's workshop, namespace your bucket with your initials or a user name to improve its uniqueness.  For example, if your name is Jane Penelope Smith, you might name the bucket, "jps-remars-workshop".  Click **Next**.
5. On the *Configure stack options* page, use the default values and click **Next**.
6. On the *Review* page, review the choices, and check the box at the bottom of the page to "acknowledge that AWS CloudFormation might create IAM resources with custom names".
7. Click **Create stack**.


This will create:

- a **VPC** with a pair of **subnets** and a **default security group** to run AWS RoboMaker instances in.
- an **S3 bucket** to store your RoboMaker assets (such as the packaged robot application).
- **Two IAM roles** that provide RoboMaker with the permissions it needs to run in your account.

The stack creation should only take a minute or two.  Once the status has changed to CREATE_COMPLETE, click on the stack's  **Outputs** tab. It will provide you with several Key/Value pairs that we will use later in the workshop.  Specifically, you should copy and paste several Key/Values to a notepad application.  This is not required, but be prepared to navigate back to the CloudFormation **Outputs** tab when you're asked for these values later.  You will need the values for the following:

- VPC
- PublicSubnet1
- PublicSubnet2
- DefaultSecurityGroupID
- RoboMakerS3Bucket

**Congratulations!** You have completed the setup portion of the workshop.

**[Continue to the next module.](../marsrover/)**
