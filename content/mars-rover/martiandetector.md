---
title: "Activity #2: Martian Detector"
chapter: true
weight: 8
---

# Find Martians with AWS Robomaker and Amazon Rekognition

![alien](../../images/mars-rover/alien.jpg)

Our objective: Find Martians!

In this exercise, we'll use our rover robot to hunt for Martians (and other objects) in our simulated world.  You'll create a new simulation in a different world, and you'll use the camera on the robot to identify objects in the scene.  You'll then review some metrics about the objects you identified with a dashboard that's being populated by the items discovered by your robot.    

When complete, you will have learned:

* How to use AWS AI services like Amazon Rekognition from your ROS application.
* How to collect data and write it to additional services, like Amazon S3.
* How to invoke ROS service commands on a robot.

## ROS Application Architecture

The ROS application we're using for this exercise is comprised of several nodes and topics.  An abbreviated node graph for our application looks like this:

   ![ros-nodes](../../images/mars-rover/ros-nodes.jpg)

   For this activity, we'll review what's happening in the nodes and topic within the dotted rectangle:
   
* *object_detector*:  This node receives images from the camera, and uses Amazon Rekognition to identify the objects in the image.  It publishes the list of objects that were found to the *detected_objects* topic. 
* *notifier*:  This node subscribes to the *detected_objects* topic, and inspects the list of objects that were found.  It writes the list of objects to a file in Amazon S3.  When it finds an alien, it uses Amazon Simple Notification Service (SNS) to send an alert, and it updates the LED display on the physical rover to indicate that an alien was found.
     
Review the system architecture.  This is what we'll be building in this activity:
 
   ![app-architecture](../../images/mars-rover/app-architecture.jpg) 


   
## Activity tasks

1. Let's review the code in the *object_detector* node.  Recall that this node is responsible for the following:

 * retrieving images from the camera.
 * sending them to Amazon Rekognition to get a list of labels that represent the objects found in the image.
 * publish the list of labels to the *detected_objects* topic.
    
    ROS nodes are commonly written in Python or C++.  While there is support for other programming languages, these two languages are the most common.  The nodes in our application are written in Python.  Open the *object-detector* node.  It will be located in your IDE at *project_root->mars-rover->robot_ws->src->martian_detector->nodes->object_detector*.

    ![object-detector-ide](../../images/mars-rover/object-detector-ide.jpg) 

    Let's review a few sections of this node.  First, let's take a look at where this node advertises to the ROS master that it's a publisher to the *detected_objects* topic.  Review the constructor for the `ObjectDetector` class:
    
    ```python
        def __init__(self):
            self.pub = rospy.Publisher('/detected_objects', String, queue_size=1)
            self.rekog = boto3.client('rekognition', AWS_REGION)
    ```
    
    The first line of the constructor tells the ROS master that this node will publish to the *detected_objects* topic, and the data type it will send is `String`.  The second line is the initialization of the Amazon Rekognition client that will be used elsewhere in the node.    
    
    Next, let's review how this node gets images from the camera.  Take a look at the `get_image` function:    
    
    ```python
        def get_image(self):
            image = None
            try:
                # Get one image from the camera and return it
                image = rospy.wait_for_message(CAMERA_TOPIC, CompressedImage, timeout=3).data
            except Exception as e:
                rospy.logerr('Error getting image from camera: %s', e)
            else:
                return image

    ```

    The `get_image` function has one purpose - to get an image from the camera.  It uses the `wait_for_message` method in the Python ROS package `rospy` to get an image from the camera.  `wait_for_message` is a simple function that subscribes to the topic provided, gets one message (a `CompressedImage` in our case) and then un-subscribes from the topic.  
    
    The `get_image` function is invoked from the `process_image` function.  Let's take a look at the  function.
    
    ```python
    def process_image(self):
        # Get picture from appropriate camera topic and send to Rekognition for labels
        rospy.loginfo('Getting image from camera')
        img = self.get_image()

        if img is None:
            # No image was returned
            rospy.logwarn('No image was returned from the camera.')

        rospy.loginfo('Sending image to Rekognition for analysis.')
        # Call Rekognition
        try:
            response = self.rekog.detect_labels(
                Image={'Bytes': img},
                MinConfidence=80,
                MaxLabels=10
            )
            labels = self.sort_labels(response)
            rospy.loginfo('Following labels detected: %s' % (labels))
        except Exception as e:
            rospy.logerr('Error processing Rekognition: %s', e)
        else:
            # Write detected lables to /detected_objects
            self.pub.publish(",".join(labels))
        
        return
    ```
    
    This function does three main tasks:
    
    * It uses the `get_image` function to get an image.  
    * It passes that image to Amazon Rekognition using the `detect_lables` method of the Rekognition client.  Notice that this method also takes a `MinConfidence` parameter.  The value provided here ensures that Rekognition does return any labels for objects where it was not "80%" confident that it was accurate in identifying the object.
    * After sorting the labels, it publishes them as a comma-delimted list on the *detected_objects* topic.

2.  Now let's take a look at the *notifier* node. Recall that this node has the following responsibilities:

    * Subscribe to the *detected_objects* topic to get a continuous stream of objects that have been detected by the rover.
    * Persist the list of detected ojects to Amazon S3.
    * Inspect the list of objects, looking for aliens.
    * When aliens are found, send notifications using Amazon SNS.

    The constructor for the `Notifier` class tells the ROS master about the topics to which this node will publish and subscribe:
    
    ```python
        def __init__(self):
            rospy.loginfo('Initializing Notifier node.')
            self.object_sub = rospy.Subscriber ('/detected_objects', String, self.notify)
            self.pub = rospy.Publisher('/led_cmds', Int64MultiArray, queue_size=1)

    ```
    We'll be subscribing to the *detected_objects* node, and publishing to the *led_cmds* topic.  The *led_cmds* topic will only have a subscriber on the physical JPL Open Source Rover (OSR) robot.  In our simulation, we'll publish to that topic but since we haven't created a simulated LED display on our robot model, we will not have a node that subscribes to it.  Notice that the line that configures the subscription to *detected_objects* passes a reference to the `self.notify` method of the `Notifier` class.  This is a callback function that gets invoked each time a message is received on the topic.  Let's take a look at the `notify` function.  This is where all the action is in this node:
    
    ```python
        def notify(self, msg): 
            
            found_objects = msg.data
            rospy.loginfo('Detected an object.  Object found: %s.', found_objects)                
                
            try:
                # log objects found to S3 for reporting
                s3.Object(s3_bucket_name, 'web/objects.txt').put(
                    ACL='public-read', 
                    Body=found_objects)
    
                led_msg = Int64MultiArray()
                
                #see if we found the search object.  if so, send alert.
                objects = found_objects.split(',')
                if search_object in objects:
                    
                    #Alien found.  Send message to LED topic so the display will update if it's being used.
                    led_msg.data = [2, 1] # ^^
                    self.pub.publish(led_msg)
    
                    #send SMS message for the alert
                    sns.publish(
                        PhoneNumber=phone_number,
                        Message='Alert!  %s detected!' % search_object)
                else:
                    #No alien found.  Publish message for simple face
                    led_msg.data = [0, 1] # -_-
                    self.pub.publish(led_msg)
    
            except Exception as e:
                rospy.logerr('Error logging or sending alert. \n\n %s', str(e))
   
    ```

    This function takes the data received in the message as input and attempts to write it to a file in S3.  It then iterates the list of objects and looks for a specific label.  When that label is found, it publishes a SMS message to a phone number using Amazon SNS.  The name of the S3 bucket, the label to search for, and the phone number to message, are all input parameters to our node.  We'll review how to set those parameters in the next step.    
         
3. Ok, let's prepare to create a new simulation where we can search for aliens with the Mars rover.  Before you launch the simulation, you'll need to configure the nodes for your environment.  In ROS, a launch file is an XML document that contains information about the nodes that will be run on your robot.  Open the launch file that we'll use in our simulation.  Open the file called `martian_detector.launch`, which can be found in your project at *project-root->mars-rover->robot_ws->src->martian_detector->launch*:
  
    ![launch-ide](../../images/mars-rover/launch-ide.jpg)



2. Create a development environment where we can develop our ROS application.  Click the hamburger menu on the upper-left of the page to expand the RoboMaker menu.  Then create a new development environment by navigating to *Development->Development environments* and choose **Create environment**.
 
    ![create-env](../../images/mars-rover/create-environment.jpg)
 
  
3. On the *Create AWS RoboMaker development environment* page, enter the following:

    * Name: `rover-workshop` or something descriptive
    * Instance type: `m4.large`
    * Choose the VPC (default), and a subnet for your development environment 
    * Click **Create**

4. This opens a new tab in your browser with the Cloud9 IDE.

    *This may take a few minutes to complete, but when the creation process has completed, you will see something similar to this:*

    ![1_cloud9](../../images/1_cloud9.png)

    The *Welcome page* provides helpful information to get started, but for now we are not going to use it, so click the *X* on the tab to close.  The IDE is broken down into four sections:

    ![1_c9_layout](../../images/1_c9_layout.png)

    - (1) The AWS RoboMaker menu provide quick access to common actions. It is updated dynamically when a file in your project is modified.  The `roboMakerSettings.json` file is used to configure your RoboMaker project, and drives the content of this menu.
    - (2) Any files and folders will reside here, and can be selected and double-clicked to open in the editor pane (#4).
    - (3) The lower section is an adjustable pane for creating or monitoring command line operations. ROS developers work in this area to build, test, and interact with local code.
    - (4) This is the main editor pane.

5. Delete the `roboMakerSettings.json` file by right-clicking on it and selecting *Delete*->Yes. We are going to clone a new project from GitHub, and it will contain this file.  So we can safely delete the default file.


6. For this activity, you will be using a command line terminal to perform several tasks associated with the development of our robot.  Throughout this guide, we'll reference *the terminal*.  When you see that, we want you to run commands in the bash shell in your Cloud9 environment (Section 3 of the IDE, as defined in the previous step).

7. The project we'll be working with is located in GitHub.  You need to clone it into the Cloud9 environment so you can work with it.  Copy and paste the following commands into the  terminal.  This will clone the repository:

    ```bash
    cd ~/environment

    # clone the DogFinder repository
    git clone https://github.com/MacInnis/mars-rover.git
    ```

8.  You will now have a new directory in your project called *mars-rover*.  Let's take a look at the contents of that folder.  There are two folders of interest in our project.  The first folder, *content* is not related to our robot code.  This contains some HTML and JavaScript files that will be used later to build a dashboard to view our robot metrics.  We'll come back to that later.  Expand the mars-rover folder (you should now be looking at *mars-rover->mars-rover*).  In this folder, we see two new folders called *robot_ws* and *simulation_ws*.  These are the workspaces for our robot application.  In ROS development, a workspace is a folder where you modify, build, and install packages.  It is common practice for robotics developers to create multiple workspaces for their system to better encapsulate the components.  In our workshop, we have a workspace for our robot code (robot_ws).  This contains all the ROS nodes, services, and any dependencies needed by the robot application.  We also have a second workspace for our simulation material (simulation_ws).  A simulation workspace typically contains the artifacts needed to run our robot in simulation.  This contains items such as the 3D model for the robot and the 3D objects and textures needed to create the world in which the robot will be simulated.  In this workshop, we're going to be working in these folders to view and modify our robot code.  

    ![roboMakerSettings](../../images/mars-rover/nav-tree.jpg)


9.  Before we can run any simulations, we need to configure the environment to use the networking and roles we created in the earlier setup activity.  To do this, edit the project settings by using the menu at the top of the IDE and navigating to *Run->Add or Edit Configurations...*.  This will open the configuration window.

    ![config](../../images/mars-rover/configuration.jpg)
     
10.  We need to tell RoboMaker what file to use to save our project configuration.  Click the **Switch config** button on the lower-left side of the window and navigate to the file *mars-rover/mars-rover/roboMakerSettings.json*.  Click **Ok**.

    ![select-config](../../images/mars-rover/select-config.jpg)

11. Expand the *Simulation* item in the left-navigation area and click on *Mars Rover Simulation*.

12. Let's configure the simulation job to use IAM role you created earlier.  When the simulation runs, it will assume this role.  This gives the robot application the permissions it needs to access other AWS resources during simulation.  To set it, scroll until you find the property **Simulation job->IAM role**.  In the drop-down, choose the role named *robomaker-simulation-role-us-west-2*.

13.  Now configure the networking configuration for the simulation.  This will provide the RoboMaker simulation environment with the information it needs to access the network.  This is required so that the robot application can gain network access to other AWS services during simulation.  Scroll until you find the property for **Simulation job->Security Groups**.  In this field, paste the value for the name/value pair for the *DefaultSecurityGroupID* property that was created in the workshop setup step earlier.  Similarly, update the value for **Simulation job->Subnets** by pasting the comma-separated values for the *PublicSubnet1* and *PublicSubnet2* values.  The networking configuration will look similar to this when complete:

    ![select-config](../../images/mars-rover/network-config.jpg)    

14.  Finally, let's configure the project to use the S3 bucket we created earlier during setup.  Note that there are **two** locations in the configuration where we need to set the S3 bucket.  The first location is located at **Robot application->S3 bucket**.  In this field, click the drop-down and select the bucket you created earlier.  The second location is located a bit further down under **Simulation application->S3 bucket**.  Choose the same bucket from the drop-down list.

15.  Before we exit the configuration, let's take a look at some of the other properties you can set.  We're not going to change these today, but let's review for future use.  In the menu on the left, notice the menu items for COLCON BUILD and COLCON BUNDLE.  Colcon is a tool that is used to compile (build) and package (bundle) ROS workspaces.  Our configuration has been pre-configured to build and bundle the workspaces used in the workshop.  Also notice the WORKFLOW menu item.  This enables you to customize any build and bundle operations you may need in your project.  For today's workshop, we have created a single workflow that will build and bundle all the workspaces.  

16.  Those are all the changes we need for now.  Click **Save** to exit the configuration window.  

17.  Ok, enough configuring already.  Let's build and bundle our robot application and see it running in simulation!  To build and bundle our robot application and simulation material, use the IDE menu and choose *Run->Workflow->Mars Rover - Build and Bundle All*.  This will kick-off the compilation and packaging of our robot and simulation workspaces.  This will take about one or two minutes to complete.  You'll see two new tabs open in the terminal area of the IDE.  You can look at these tabs to see the log outputs of the build and bundle operations.  When both tabs have logged an entry for `Process exited with code: 0`, then the build and bundle operations are complete.  The reslting output of these operations is two .tar files containing our robot application, and our simulation artifacts.  For reference, the files for the robot workspace and the simulation workspace are saved to `mars-rover/robot_ws/bundle/output.tar` and `mars-rover/simulation_ws/bundle/output.tar` respectively.   

18.  Kick off the simulation job by using the IDE menu and choosing *Run->Launch Simulation->Mars Rover Simulation*.  You'll notice another tab open in the terminal section of the IDE.  You can witness the IDE copying the .tar files mentioned above to S3, and then the simulation job will be created.  In the IDE menu, you'll see the menu indicate that the simulation is being prepared.

    ![sim-preparing](../../images/mars-rover/sim-preparing.jpg)
 
    When the simulation is ready and running, the menu will change to *Running*. 

19. Once it is running, we can now interact with the simulation.  AWS RoboMaker includes several common robot simulation tools to interact with your robot.  In this step, we'll use the open source tool [Gazebo](http://gazebosim.org/) to interact with the robot.  In the IDE menu, choose *Simulation (Running)->Applications->Gazebo*.  This will open a new window where you will see the Mars rover in a Martian environment.
 
    ![rover-world](../../images/mars-rover/rover-world.jpg)

    You can change the zoom level and view angle to obtain a better view of the rover and the environment.  Note, that this is *much* easier to do with a mouse rather than a laptop touchpad.  You can zoom with a mouse wheel (our touchpad equivalent).  To adjust the positioning of the view, click and drag in the world.  To adjust the angle of view, shift-click and drag.  If you have issues and you lose sight of the rover, you can reset the view using the Gazebo menu.  To reset, click *Edit->Reset World*, and then *Camera->Reset View Angle*.

20.  When a robot is running in the simulation environment, we can connect to it to inspect it, control it, and debug it.  To do this, we're going to use a terminal that will be connected to the robot application in the simulation.  From this terminal, we'll be able to view the ROS topics and messages, as well as send specific commands to the robot.  To open the terminal, go back to the Cloud9 IDE menu, and choose *Simulation (Running)->Applications->Terminal*.  This will open a new window containing a terminal connected to the robot.  Adjust your Gazebo window and your Terminal window so you can see each comfortably.  Also adjust the view in Gazebo so you can see the rover in a wider landscape.  This will make it easier to see the robot while it moves.

    ![gazebo-with-terminal](../../images/mars-rover/gazebo-with-terminal.jpg)

    Let's take a look at some of the ROS information for our robot.  To see all the ROS topics that are being published, issue the following command in the terminal:
    
    ```text
    rostopic list
    ```

    *Note*: To paste commands in the Terminal window, use `Shift+Ctrl+V`, or use the *Edit->Paste* menu item.

    You will see a long list of topics.  One of the topics, `/rosout`, contains logging information from other nodes in the system.  To see the log data currently being generated by the robot, issue the following command in the Terminal:
    
    ```text
    rostopic echo /rosout
    ```

    You'll see lots of data quickly scrolling through the terminal.  When you've had enough, stop echoing the topic by pressing `Ctrl-C` in the terminal.

21.  Let's make this thing move!  We're going to use our keyboard to control the robot in the simulation environment.  To do so, we'll run a script that has been deployed with our robot that takes keyboard input and converts it to motion-control messages that are sent to the robot.  To run this script, use the same terminal window from the previous step and issue the following command:

    ```text
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
    
    Use the keys displayed in the terminal to move the robot. For example to move forward, press `i`, to move clockwise, press `o`.  You don't need to press and hold - you only need to press the key once to send that instruction to the robot.  By default, the rover will move rather slowly.  Pres the `q` key on your keyboard several times to increase the speed.  Increase the speed to approximately 6, and then issue another forward (`i`) command to see it move faster.  Continue experimenting with the controls to move the rover.  
    
    Did you know that the gravity on mars is only about 1/3 of that on earth?  In the Gazebo simulation, the gravity has been reduced to simulate Mars.  If you try to corner your rover too quickly, you can see the results of low gravity and angular momentum.  Try increasing the speed to about 18 to 20, and then attempt to turn (`o`) your rover.  What happens?


## Activity wrap-up

In this activity, you used AWS RoboMaker to build and simulate a robot that you can control with a keyboard controller.  You viewed some of the ROS information for your robot, and you learned how to interact with the robot in the simulation environment.  

In the next activity, you'll extend the robot application to use AWS AI services to detect Martians in a new world! 

[Proceed to the next activity](../martiandetector)
