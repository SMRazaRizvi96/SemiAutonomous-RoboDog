#!/usr/bin/env python

# Python libs
import sys
import time
import math
import random

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import smach
import smach_ros

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


VERBOSE = False

# define state Play
class Play(smach.State):

    """
    In PLAY state the robot has to use its camera to track and follow the green ball.
    In this state, the image received from the Robot's camera is processed using Open CV to detect any contours of the green ball.
    If any contour is found, the Robot is directed to go near the ball, otherwise the Robot is directed to rotate, and find the ball.
    Once the Robot has reached near the ball, the robot rotates its head right, and left upto 45 degrees and then again looks for the ball.
    The robot searches for the ball for 10 seconds. This functionality is implemented using a timer.
    """
    
    def __init__(self): 
        
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['after finishing play time'])

	self.head_pub = rospy.Publisher("/robot/joint1_position_controller/command",
                                        Float64, queue_size=1)
        
    def execute(self, userdata):
      
        rospy.loginfo('Executing state PLAY')
	radius = 0
	elapsed = 0
	data = Float64()
	while(elapsed < 10):
		#reached = 0
		while(rospy.get_param("detectBallFlag")):
		
			image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

			greenLower = (50, 50, 20)
			greenUpper = (70, 255, 255)

			blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(hsv, greenLower, greenUpper)
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
			#cv2.imshow('mask', mask)
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
						cv2.CHAIN_APPROX_SIMPLE)
			cnts = imutils.grab_contours(cnts)
			center = None
			if len(cnts) > 0:
				#print 'Radius Received: ', radius
				c = max(cnts, key=cv2.contourArea)
				((img_x, img_y), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				#print 'Calculated Radius: ', radius
				#print 'Center: ', center[0]
				
				if (395 <= center[0] <= 405 and radius > 100):
					#reached = 1
					print 'Reached'
					vel.angular.z = 0.0
					vel.linear.x = 0.0
					pub.publish(vel)
					data.data = 1
					self.head_pub.publish(data)
					time.sleep(3)
					data.data = -1
					self.head_pub.publish(data)
					time.sleep(3)
					data.data = 0
					self.head_pub.publish(data)

				# only proceed if the radius meets a minimum size
				elif (radius > 10):
					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					cv2.circle(image_np, (int(img_x), int(img_y)), int(radius),
						   (0, 255, 255), 2)
					cv2.circle(image_np, center, 5, (0, 0, 255), -1)
					vel = Twist()
					vel.angular.z = 0.004*(center[0]-400)
					vel.linear.x = -0.03*(radius-100)
					pub.publish(vel)
					
				else:
					vel = Twist()
					vel.linear.x = 0.5
					pub.publish(vel)
			cv2.imshow('window', image_np)
			cv2.waitKey(2)
			timer = time.clock()

		elapsed = time.clock() - timer
		#print 'Time elapsed in Finding the ball: ', elapsed , ' Seconds'
		vel.angular.z = 0.7
		vel.linear.x = 0.0
		pub.publish(vel)

	print 'Searched for 10 seconds but couldnt find the ball'
        return 'after finishing play time'

def detectBall(image):

	"""
	This function is the callback of the Camera Image topic.
	This function uses Open CV to process the image received, and detects the contours of Green ball in that image.
	If any contour is found, it turns the Parameter "detectBallFlag" to 1, otherwise turns it to 0.
	"""
	
	global radius, center, image_np, np_arr, cnts

	if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = 0
        # only proceed if at least one contour was found
        if len(cnts) > 0:
	    rospy.set_param("detectBallFlag", 1)

	else:
	    rospy.set_param("detectBallFlag", 0)



# define state Normal
class Normal(smach.State):

    """
    In NORMAL state a random number times (from 1 to 5) random coordinates were generated within the area.
    These coordinates were then passed to the go_to_goal() function, which takes care of moving the robot to the generated random coordinate. 
    This function also checks on the parameter "detectBall Flag", if the flag is 1, this means that the ball was detected, and then it transitions to the PLAY state.
    If no ball was detected, it then transitions to the SLEEP state after completing this behavior.
    """
    def __init__(self):
        
        smach.State.__init__(self, 
                             outcomes=['after finishing normal behavior',
 				       'ball detected'])

    def execute(self, userdata):

	rospy.loginfo('Executing state NORMAL')
	rospy.set_param("state", 'normal_state')
	print 'Current State in NORMAL:' ,rospy.get_param("state")
	#while not rospy.is_shutdown():      
	goal = Point()
	i = random.randint(1,5)
	for x in range(i):
		goal.x = random.uniform(-6, 7)
		goal.y = random.uniform(-7, 7)
		print 'goal generated in Normal state: ', goal.x,' ', goal.y
		go_to_goal(goal)
		if rospy.get_param("detectBallFlag") == 1:
			return 'ball detected'
	return 'after finishing normal behavior'

def go_to_goal(goal):

	"""
	This function generates the velocity commands to move the robot to the coordinate received in the parameter.
	While the robot is not aligned to the Goal the robot keeps rotating. Once the robot is aligned to the goal, linear velocity commands are generated in that direction until it reaches the goal under some threshold.
	This function also keeps checking the parameter "detectBallFlag", and if this parameter is 1 and the robot is in the NORMAL state, the function stops, because this means that now the robot is not required to go to the given coordinate, and the robot just needs to follow the ball, and the function is no longer needed.
	"""
	
        	   
        #print 'going to goal: ', goal.x, goal.y 
	speed = Twist()
	speed.linear.x = 0.0
        speed.angular.z = 0.0
	time.sleep(2)
	inc_x = goal.x -x
        inc_y = goal.y -y

	angle_to_goal = math.atan2(inc_y, inc_x)
	
        #print get_active_states()
	
	aligned = 0
	while abs(angle_to_goal - theta) > 0.15 or math.sqrt(pow(goal.x-x, 2) + pow(goal.y-y, 2)) > 0.2:
    	    
	    if (rospy.get_param("detectBallFlag") == 1 and rospy.get_param("state") == 'normal_state'):
		speed.linear.x = 0.0
        	speed.angular.z = 0.0
	   	pub.publish(speed)
    	        return

	    inc_x = goal.x -x
            inc_y = goal.y -y
	    
            angle_to_goal = math.atan2(inc_y, inc_x)
	    #print 'Angle to Goal: ', angle_to_goal
	    #print 'Theta: ', theta

	    
    	    if (abs(angle_to_goal - theta) > 0.15):
        	speed.linear.x = 0.0
        	speed.angular.z = -0.7*(angle_to_goal - theta)
		if (abs(angle_to_goal - theta) <= 0.15):
		   aligned = 1

	    else:
		#aligned = 1
       		speed.linear.x = 0.5
        	#speed.angular.z = -0.7*(angle_to_goal - theta)
		speed.angular.z = 0.0
	    pub.publish(speed)
	
	print 'Reached goal: ', goal.x, goal.y 
	speed.linear.x = 0.0
	speed.angular.z = 0.0
	pub.publish(speed)

def newOdom(currentPos):
    """
    This function is a callback for the robot's odometry topic "/robot/odom".
    This function extract the x,y coordinate of the Robot, and also uses the quaternion orientation to compute the Euler orientation 'Theta' of the Robot with respect to the world_frame.
    """
    
    global x,y, theta
    x = currentPos.pose.pose.position.x
    y = currentPos.pose.pose.position.y

    rot_q = currentPos.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    



# define state Sleep
class Sleep(smach.State):

    """
    In SLEEP state the robot goes to the coordinate (0,0) and stays there for a random number of seconds which ranges from 4 to 7 seconds.
    """

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['after finishing sleep time'])
        
    def execute(self, userdata):
         
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state SLEEP')
	rospy.set_param("state", 'sleep_state')
	print 'Current State in SLEEP:' ,rospy.get_param("state")
	
        #sub = rospy.Subscriber("/robot/odom", Odometry, newOdom)
	sleep_pos = Point()
	sleep_pos.x = 0
	sleep_pos.y = 0
	time.sleep(3)
	go_to_goal(sleep_pos)
	print 'Sleeping... ZzzZZZzz'

	time.sleep(random.randint(4,7))
	
	return 'after finishing sleep time'

        
def main():

    """
	This is a State Machine for a Robot Dog name wheely.
	The main function initializes the ros node, subscribes to the topic: robot's odometry topic ("/robot/odom")
	and the robot's camera image topic ("/robot/camera1/image_raw/compressed"), and creates a publisher for the robot's velocity commands on the topic "/robot/cmd_vel".
	
	This function also intializes the "detectBallFlag" and "state" parameters in the ROS Parameter Server.

	This function also defines the state machines and adds three states/behaviors to the container.
    """
    
    rospy.set_param("detectBallFlag", 0)
    rospy.set_param("state", 'none')
    
    global pub
    sub = rospy.Subscriber("/robot/odom", Odometry, newOdom)
    subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, detectBall,  queue_size=1)

    pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size = 1)

    rospy.init_node('Assignment_2_State_Machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'after finishing normal behavior':'SLEEP',
					    'ball detected': 'PLAY'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'after finishing sleep time':'NORMAL'})
	smach.StateMachine.add('PLAY', Play(), 
                               transitions={'after finishing play time':'NORMAL'})


    # Create and start the introspection server for visualization. We 
    # can visualize this is smach viewer
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine 
    smach_ros.set_preempt_handler(sm)
    
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application 
    
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
