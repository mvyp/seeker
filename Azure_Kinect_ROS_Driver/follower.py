#!/usr/bin/env python3

import rospy
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import copysign
from visualization_msgs.msg import MarkerArray
import azure.cognitiveservices.speech as speechsdk

class Follower():
    def __init__(self):
        rospy.init_node("follower")
        
        speech_key, service_region = "80c72f7522eb4105aecaa9766104bd53", "eastus"
        speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=service_region)

        # Set the voice name, refer to https://aka.ms/speech/voices/neural for full list.
        speech_config.speech_synthesis_voice_name = "en-US-AriaNeural"

        # Creates a speech synthesizer using the default speaker as audio output.
        self.speech_synthesizer = speechsdk.SpeechSynthesizer(speech_config=speech_config)   
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # The dimensions (in meters) of the box in which we will search
        # for the person (blob). These are given in camera coordinates
        # where x is left/right,y is up/down and z is depth (forward/backward)
        self.min_x = rospy.get_param("~min_x", -0.2)
        self.max_x = rospy.get_param("~max_x", 0.2)
        self.min_y = rospy.get_param("~min_y", -0.3)
        self.max_y = rospy.get_param("~max_y", 0.5)
        self.max_z = rospy.get_param("~max_z", 1.2)
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.8)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.025)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.025)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight left/right displacement of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 2.0)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 5.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.05)
        
        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)
        
        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the point cloud
        self.marker_subscriber = rospy.Subscriber('/body_tracking_data', MarkerArray, self.set_cmd_vel, queue_size=1)

        rospy.loginfo("Subscribing to MarkerArray...")

        rospy.loginfo("Ready to follow!")
        self.speech_synthesizer.speak_text_async("Ready to follow!").get()
    def set_cmd_vel(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0
        
       # If we have points, compute the centroid coordinates
        if msg and msg.markers[2].pose.position.z<2:
            #z distance x leftright y updown
            x = msg.markers[2].pose.position.x 
            y = msg.markers[2].pose.position.y
            z = msg.markers[2].pose.position.z
            
            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold):
                # Compute the angular component of the movement
                linear_speed = (z - self.goal_z) * self.z_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.linear.x = copysign(max(self.min_linear_speed, 
                                        min(self.max_linear_speed, abs(linear_speed))), linear_speed)
            else:
                self.move_cmd.linear.x *= self.slow_down_factor
                
            if (abs(x) > self.x_threshold):     
                # Compute the linear component of the movement
                angular_speed = -x * self.x_scale
                
                # Make sure we meet our min/max specifications
                self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                        min(self.max_angular_speed, abs(angular_speed))), angular_speed)
            else:
                # Stop the rotation smoothly
                self.move_cmd.angular.z *= self.slow_down_factor
                
        else:
            # Stop the robot smoothly
            self.move_cmd.linear.x *= self.slow_down_factor
            self.move_cmd.angular.z *= self.slow_down_factor
            self.speech_synthesizer.speak_text_async("I can not find you.Please step back!").get()
        # Publish the movement command
        self.cmd_vel_pub.publish(self.move_cmd)
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.speech_synthesizer.speak_text_async("Stopping the robot...").get()
        # Unregister the subscriber to stop cmd_vel publishing
        self.marker_subscriber.unregister()
        
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)        
                   
if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")

