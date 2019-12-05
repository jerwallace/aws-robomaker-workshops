---
title: "Troubleshooting Guide"
chapter: true
weight: 13
requiredTimeMins: 15
description: If you encounter issues as you work through the workshop, please see this guide.
---

# Troubleshooting Guide
Please see below for some common issues and how to solve them.

**

**Unable to Locate Docker Container When Running docker run Command**

Either you forgot to run configure_docker.sh or the script was run while the Cloud9 environment was updating its dependencies.  The script can be found under jetbot/assets/scripts and needs to be run again.

**Unable to Connect to Jetbot Over WiFi**

There are a few reasons that you are unable to establish connectivity:

* Do you have a VPN enabled?  You must be on the same wireless network as the robot in order to connect to it and cannot be running a VPN client.

*  You can connect to the robot using a micro USB cable.  Once connected, your laptop will have a new network connection available on the IP 192.168.55.100 -- you can connect to the robot by visiting **192.168.55.1**