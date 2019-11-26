---
title: "Activity #2: Martian Detector"
chapter: true
weight: 10
description: "In this activity you will learn how to integrate a robot application with AWS services.  You'll use a simulation environment to test your robot application and view the results on a dasboard."
---

# Find Martians with AWS Robomaker and Amazon Rekognition

![alien](../../images/mars-rover/alien.jpg)

Our objective: Find Martians!

In this exercise, we'll use our rover robot to hunt for Martians (and other objects) in our simulated world.  You'll create a new simulation in a different world, and you'll use the camera on the robot to identify objects in the scene.  You'll then review some metrics about the objects you identified with a dashboard that's being populated by the items discovered by your robot.    

When complete, you will have learned:

* How to use AWS AI services like [Amazon Rekognition](https://aws.amazon.com/rekognition) from your ROS application.
* How to collect data and write it to additional services, like [Amazon S3](https://aws.amazon.com/s3).
* How to invoke ROS service commands on a robot.

## ROS Application Architecture

The ROS application we're using for this exercise is comprised of several nodes and topics.  An abbreviated node graph for our application looks like this:

   ![ros-nodes](../../images/mars-rover/ros-nodes.jpg)

   For this activity, we'll review what's happening in the nodes and topic within the dotted rectangle:
   
* *object_detector*:  This node receives images from the camera, and uses Amazon Rekognition to identify the objects in the image.  It publishes the list of objects that were found to the *detected_objects* topic. 
* *notifier*:  This node subscribes to the *detected_objects* topic, and inspects the list of objects that were found.  It writes the list of objects to a file in Amazon S3.  When it finds an alien, it uses Amazon Simple Notification Service (SNS) to send an alert, and it updates the LED display on the physical rover to indicate that an alien was found.
     
Review the system architecture.  This is what we'll be building in this activity:
 
   ![app-architecture](../../images/mars-rover/app-architecture.jpg) 

When complete, you'll navigate your rover in the simulated world, similar to the previous exercise.  The data generated from the rover will populate a dashboard similar to this:

   ![dashboard](../../images/mars-rover/dashboard.jpg) 


   
## Activity tasks

1. Before we start reviewing the ROS application and simulation, there is one housekeeping task that needs to be completed.  As mentioned above, we'll be reviewing some of the data captured by the rover in a dashboard.  The dashboard is comprised of some static HTML and JavaScript.  This code was downloaded to the Cloud9 IDE environment when you cloned the project from GitHub in the previous activity.  Before you can view your dashboard, you need to copy the HTML and JavaScript to your S3 bucket.  Find the S3 bucket name for the bucket you created as part of the CloudFormation stack created in the workshop setup.  If you didn't take note of the bucket name, you can find it in the **Outputs** section of your CloudFormation stack.  Once you have the bucket name, replace <s3_bucket_name> in the command below with your bucket name (be sure to replace the < and > characters as well), and run the command in the Bash shell of your IDE:

    ```text
    aws s3 cp ~/environment/aws-robomaker-sample-application-open-source-rover/web/ s3://<s3_bucket_name>/web/ --recursive --acl public-read
    ```
 

2. Now let's look at the robot code.  Review the code in the *object_detector* node.  Recall that this node is responsible for the following:

 * retrieving images from the camera.
 * sending them to Amazon Rekognition to get a list of labels that represent the objects found in the image.
 * publish the list of labels to the *detected_objects* topic.
    
    ROS nodes are commonly written in Python or C++.  While there is support for other programming languages, these two languages are the most common.  The nodes in our application are written in Python.  Open the *object-detector* node.  It will be located in your IDE at *project_root->aws-robomaker-sample-application-open-source-rover->robot_ws->src->martian_detector->nodes->object_detector*.

    ![object-detector-ide](../../images/mars-rover/object_detector.png) 

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
                MinConfidence=50,
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
    * It passes that image to Amazon Rekognition using the `detect_lables` method of the Rekognition client.  Notice that this method also takes a `MinConfidence` parameter.  The value provided here ensures that Rekognition does return any labels for objects where it was not "50%" confident that it was accurate in identifying the object.
    * After sorting the labels, it publishes them as a comma-delimted list on the *detected_objects* topic.

3.  Now let's take a look at the *notifier* node. Recall that this node has the following responsibilities:

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
         
4. Ok, let's prepare to create a new simulation where we can search for aliens with the Mars rover.  Before you launch the simulation, you'll need to configure the nodes for your environment.  In ROS, a launch file is an XML document that contains information about the nodes that will run on your robot.  Let's open the launch file that we'll use in our simulation.  Open the file called `martian_detector.launch`, which can be found in your project at *project-root->aws-robomaker-sample-application-open-source-rover->robot_ws->src->martian_detector->launch->martian_detector.launch*:
  
    ![launch-ide](../../images/mars-rover/launch.png)

    We need to update the parameters for the *notifier* node.  In the XML, look for the following section and update the parameters as directed below:
    
    ```xml
    	<node name="notifier" pkg="martian_detector" type="notifier" >
    	    <param name="aws_region" value="$(arg aws_region)" />
    	    <param name="s3_bucket_name" value="<your-bucket-name>" />
    	    <param name="phone_number" value="<your-phone-number>" type="str" />
    	    <param name="search_object" value="Alien" />
       	</node>
     ```
    
    - **s3_bucket_name**: Set the value to the S3 bucket name created during the workshop setup.  If you didn't note the bucket name earlier, you can find it in the **Outputs** section of the CloudFormation stack you created.
    - **phone_number**:  Set this to  your mobile phone number.  You will receive SMS messages at this number.  Be sure to format it with the + sign, country code, and number.  For example:  +12065551212.

    Be sure to save (Ctrl-s) the file after you change it.

5. We also need to update the confiuration to tell the simulation to use a different launch file for the simulation world.  This new world will have objects and aliens scattered throughout the landscape.  To update the launch file for the simulation world, use the IDE menu to configure it.  In the menu, choose *Run->Add or Edit Configurations...* and expand the SIMULATION item on the left menu.  Click on the *Mars Rover Simulation* to view its details.  Find the *Simulation application* section, and change the value for **Launch file** to `main_with_objects.launch`.  Click **Save**.

5. Before we can run the simulation again, we need to build and bundle the robot application.  This will ensure our changes to the launch file get packaged in our robot application for the simulation.  To build and bundle the robot application, use the menu in the IDE and choose:  *Run->Workflow->Mars Rover - Build and Bundle Robot*.  This will create a new bundle containing the changes we made to the launch file.  It will take only a few seconds to complete.

6. Now we need to run the simulation with our changes.  To run the simulation, use the menu in the IDE and choose: *Run->Launch Simulation->Mars Rover Simulation*.  Wait for the menu to change to *Simulation (Running)*.

7. Similar to the previous activity, connect to your simulation using Gazebo and Terminal (*Simulation (Running)->Applications->Gazebo* and *Simulation (Running)->Applications->Terminal*) respectively.

8.  We're going to use an additional tool for this exercise.  We'll use *rqt*, a tool that enables us to inspect the data on the topics in our robot application while it's running.  We'll use *rqt* to get a near real-time view from the camera on our rover.  To open rqt, use the IDE and choose *Simulation (Running)->Applications->rqt*.

9.  To see the view from the rover's camera, we need to use the Visualization feature of *rqt* and choose the topic on which images are being published on our robot.  In rqt, enable the visualization plugin by choose from the menu:  *Plugins->Visualization->Image View*. 
  
    ![rqt-image-view](../../images/mars-rover/rqt-image-view.jpg)  
  
10.  To view the images, we need to choose the topic name where the images are published.  In the Image View window, choose the topic from drop-down list:  */camera/image_raw/compressed*.  You'll see a gray square, and not the Martian landscape.  Why is this?  It's because the default position of our camera is pointing directly up, into the sky.  We'll fix that shortly.  
  
      ![rqt-gray](../../images/mars-rover/closeup.png)
      
11. Before we adjust the camera angle, it's best if you arrange your application windows so you can see all three applications concurrently.  Also adjust the view in Gazebo so you have a relatively close-up view of the rover.  Arrange them similar to this:

      ![rover-3-window-view](../../images/mars-rover/rover-3-window.png)
      
12. We need to use two Terminal connections to our robot, so let's open a new tab on the terminal window.  This will open a second connection to the robot.  Our first terminal window will be used to move the rover, similar to the earlier exercise.  The second terminal will be used to send commands to the rover to take a picture for analysis.  To open the second terminal, use the terminal's File menu:  *File->Open Tab*.

      ![terminal-new-tab](../../images/mars-rover/terminal-new-tab.jpg)   
      
13. To move the rover in the simulation, use the teleop_twist_keyboard control, similar to the previous exercise.  To start the keyboard control, run the following command in one of the Terminal tabs:
    
    ```
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

    **Note:** The Terminal window and the tab where you ran the above command must be the active window to use the keyboard controller to move the rover.  Also recall that you should increase the speed of the rover to about 6 (press the `q` key on your keyboard several times).

14.  Now have some fun!  Move the robot around the world and look for objects in the environment. To detect a Martian, you will likely have to pretty close to the object.  Move the rover so that the alien takes up a significant portion of the frame, similar to:
    
    ![alien-rqt](../../images/mars-rover/closeup.png)

    Did you discover a Martian?  If so, you will receive a text message for each time you discover an alien (note, if you discover the same alien multiple times, you will receive multiple text messages).
    
    To continue exploring the world, change to the keyboard controller tab in the terminal window, and move around as you did previously.  When you're ready to analyze another object, switch tabs again and issue the message to take a photo and analyze it (*Tip:* Press the Up-Arrow on your keyboard to use the last command from your history.  Hit enter.)

15.  While you find objects, the rover is writing to a file in S3 with the results of what it discovered.  You can view this data on a dashboard.  To view the dashboard, replace `<s3_bucket_name>` with the name of your bucket (same bucket name as step 1 in this activity), and open the URL in a browser:     

    ```text
    https://s3-us-west-2.amazonaws.com/<s3_bucket_name>/web/index.html
    ```
16.  You should now be able to move the rover around the Martian world, searching for objects.  As you find them, your dashboard will be updated in real-time.  

      ![sim-with-dash](../../images/mars-rover/sim-w-dash.jpg)

## Activity wrap-up

In this activity, you used AWS RoboMaker to build and simulate a robot that sends images from its camera to the cloud for analysis.  You learned how ROS nodes can use AWS services for data storage and ML.  

If you're running this workshop in your account, and you want to ensure there are no continuing charges in your account, you should clean up the resources we created.

**[Proceed to account clean up](../cleanup/)**
