---
title: "Activity #3: Deploy an ML Model for Object Detection"
chapter: true
weight: 6
requiredTimeMins: 15
description: In this activity you will use a sample bundle and ML model to deploy an object detection model using Greengrass.
---

# Deploy an ML Model for Object Detection

This part of the workshop is a continuation of the existing workshop, however it can be run independently provided that you complete just the "Create a Robot" part of Activity #4.  This is needed so that we have a Greengrass group to work with for configuring the Machine Learning model as a resource.


### Copy Workshop Assets into S3

### Create Robot Application with Bundle


### Configure the Post-Deployment Lambda Function ###
In the [AWS Greengrass console](https://console.aws.amazon.com/greengrass/):

1. Click **Groups**

1. Click on the group created by **Robomaker** when you created a robot in the previous step.

1. Click **Resources**

1. Under the **Local** tab > **Add a local resource**

1. Set the Resource name as **ggModelPath** and **Device** for Resource type

1. Set the Device path to **/home/ggc_user/**

1. Leave **No OS group** set

1. Choose the Lambda function **mlModelSync**

### Add the Local Path for Model Deployment ###
In the [AWS Greengrass console](https://console.aws.amazon.com/greengrass/):

1. Click **Groups**

1. Click on the group created by **Robomaker** when you created a robot in the previous step.

1. Click **Resources**

1. Under the **Local** tab > **Add a local resource**

1. Set the Resource name as **ggModelPath** and **Device** for Resource type

1. Set the Device path to **/home/ggc_user/**

1. Leave **No OS group** set

1. Choose the Lambda function **mlModelSync** and **Read and write access**

### Configure the Model as a Greengrass Resource ###
In the [AWS Greengrass console](https://console.aws.amazon.com/greengrass/):

1. Click **Groups**

1. Click on the group created by **Robomaker** when you created a robot in the previous step.

1. Click **Resources**

1. Click the **Machine Learning** tab > **Add a machine learning resource**

1. Set the Resource name as **mlModels** and **Upload a model in S3** for Model Source

1. Set the Local path to **/home/ggc_user/ml_model**

1. Leave **No OS group** set

1. Choose the Lambda function **mlModelSync** and **Read and write access**

