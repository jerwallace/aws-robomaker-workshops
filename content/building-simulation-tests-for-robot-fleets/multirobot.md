---
title: "Activity #2: Launch Multiple Robots in a Fleet Simulation"
chapter: true
weight: 8
description: In this activity you will explore other methods for building and deploying applications in the development environment, and see how an application can directly interact with AWS services in a ROS native manner (nodes and topics), and how any AWS service can be used through normal SDK calls (python boto3 in this instance).
---

# Cloud service integration to do object detection

## Launch Multiple Robots

## Activity wrap-up

In this activity, you built and simulated a robot application that not only interacts (turns) in Gazebo, but also utilizes other AWS services. This include ROS native integration such as CloudWatch logs and Kinesis Video Streams, or through the flexible use of AWS SDK's such as boto3 to interact with other AWS services such as Amazon Rekognition.

What was covered:

* Working with ROS applications inside the Development Environment at a command line level
* Understanding the resources required to compile (build) and package (bundle) a ROS application
* Using the AWS CLI to interact with AWS RoboMaker to create applications, upload bundles, and launch simulation jobs
* Using AWS services such as Kinesis Video Streams and CloudWatch logs directly from ROS to interact with the robot (virtual or real)
* Use standard SDK's to interact with other AWS services

### Clean-up

In this activity, you created a Development environment, CloudWatch logs, and S3 objects that incure cost. Please follow the clean-up steps in the main. README document on how to remove these and stop any potential costs for occurring.
