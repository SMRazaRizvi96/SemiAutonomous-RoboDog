# SemiAutonomous-RobotDog

This is a finite state machine for a Robot Dog

This is the 'Assignment 2 for the course 'Experimental Robotics'.

#### How to Run:

    Go to the 'Master' branch and Clone the repository
    
    Copy the 'exp_assignment2' folder into the src folder of your ROS workspace
    
    Open the terminal and run 'catkin_make' inside your ROS workspace
    
    Open the Terminal and go to the 'Scripts' folder of this package

    Give running permissions to it with $ chmod +x assignment2.py

    Run the launch file $ roslaunch exp_assignment2 gazebo_world.launch
    
    Publish position coordinates for the Ball on the topic "reaching_goal/goal"
    
**************************    

#### Gazebo Models:

The models used were defined in the XACRO and URDF formats.

##### Robot Model:

The Robot is a Dog modeled inside the robot.xacro file given in the urdf folder.
It has a base, neck, and a head with revolut joint and an rgb8 camera no its top.
The Robot Dog has two continuos wheel at the back, and a caster wheel at the front.

The Robot uses a differential drive controller for the two wheels at the back and a Joint state controller for the Head.

##### Ball Model:

The Ball is a Green Ball modeled inside the ball.xacro file given in the urdf folder.
It uses the the object controller for the motion.

##### Human Model:

The Human model is defined inside the human.urdf file given in the urdf folder.
It is a normal colorless human being sitting on a chair.

**************************    

#### Files List:

    assignment2.py: This is the State Machine
    go_to_point_ball.py: This is the action server responsible for moving the ball to the coordinate received on the topic "reaching_goal/goal"

#### ROS Parameters Used in the State Machine:

    'state': This parameter tells the current state of the Robot
    'detectBallFlag': This parameter is a flag that is 1 if the ball is detected and 0 otherwise
    
#### ROS Topics Used in the State Machine:

#####   Subscribed to:

    '/robot/odom' : Topic to get the Robot's odometry data
    'robot/camera1/image_raw/compressed' : To get the Compressed Camera Image
          
#####   Publishing to:

    '/robot/cmd_vel' : To publish velocity command to the Robot
    '/robot/joint1_position_controller/command' : To publish commands to rotate the head
	
**************************

#### ROS Action Servers Used:

    '/reaching_goal' : This is defined in the go_to_point_ball.py.
	
It subscribes to the topic "reaching_goal/goal", and the robot's odometry topic '/ball/odom' and generates velocity commands for the ball on the topic '/ball/cmd_vel' to move the ball to reach that position coordinate. It also sets the ball link by publishig on the topic '/gazebo/set_link_state'.
          
    
**************************

#### Implementation Details of the State Machine:

In this Assignment, we have developed a Finite State Machine for a Robot dog that has three states:

1. NORMAL
2. PLAY
3. SLEEP


##### 1. NORMAL:

In this state, the robot has to move on random locations.
For this, the Robot first generates a random number between 1 and 5.
This is the number of Random locations the Robot will go to.
Next, a FOR loop starting from 1 till this random number, is started to repeat the Robot movement.
In every iteration of the FOR loop, first the 'detectBallFlag' parameter is checked.
If the 'detectBallFlag' is 1, the outcome 'ball detected' is returned and the Robot transitions to PLAY state.
Otherwise, the Robot generates a random coordinate, and passes that coordinate to the the go_to_goal() function which takes care of generating the velocity commands for the Robot to reach that coordinate.
After completing the for loop, if the 'detectBallFlag' parameter remains 0, the robot transitions to the SLEEP state by returning 'after finishing normal behavior'
    
    
##### 2. PLAY:

In this state, the robot has to follow the green ball.
In this state, if the 'detectBallFlag' parameter is 1, the image received from the Robot's camera is processed using Open CV to detect any contours of the green ball.
If any contour is found, the center and radius of that contour is found. If the radius is greater than 100, and the center is within 395 - 405, we declare that the robot has reached near the ball. Otherwise, if the radius is not in this range, the velocity commands are generated such that to rotate the robot to align and bring the center and radius within the range.
Once the Robot has reached near the ball, the robot stops and rotates its head right for 3 seconds, then left for 3 seconds, upto 45 degrees and then looks straight again to find the ball.
If the robot is unable to find the ball, the Robot is directed to rotate, and look for the ball.
If the robot is not able to find the ball for 10 seconds, the state switches back to the NORMAL state by returning 'after finishing play time'.

    
##### 3. SLEEP:

In this state, the robot goes to the coordinate (0,0) and stays there for 4 to 7 seconds.
In this state, just like in the NORMAL state, this coordinate is passed to the go_to_goal() function which takes care of generating the velocity commands for the Robot to reach that coordinate.
After completing this state, the robot switches back to the NORMAL sate by returning 'after finishing sleep time'.


**************************

#### Functions Used in the State Machine:

##### newOdom():
This function is a callback for the robot's odometry topic "/robot/odom".
This function extract the x,y coordinate of the Robot, and also uses the quaternion orientation to compute the Euler orientation 'Theta' of the Robot with respect to the world_frame.
        
##### detectBall():
This function is the callback of the Camera Image topic "robot/camera1/image_raw/compressed".
This function uses Open CV to process the image received, and detects the contours of the Green ball in that image.
If any contour is found, it sets the Parameter "detectBallFlag" to 1, otherwise sets it to 0.

##### go_to_goal():
This function generates the velocity commands to move the robot to the coordinate received in the parameter.
While the robot is not aligned to the Goal the robot keeps rotating. Once the robot is aligned to the goal, linear velocity commands are generated in that direction until it reaches the goal within some threshold.
This function also keeps checking the parameter "detectBallFlag", and if this parameter is 1 and the robot is in the NORMAL state, the function stops, because this means that now the robot is not required to go to the given coordinate, and the robot just needs to switch to the PLAY state and follow the ball.

**************************

#### Limitations:
	
1. Currently the robot could only move around within a limited space, and only in 2D.
2. The Robot can only detect and follow a green ball within a certain range of HSV.

**************************

#### Authors:

Laiba Zahid (S4853477): S4853477@STUDENTI.UNIGE.IT

Syed Muhammad Raza Rizvi (S4853521): S4853521@STUDENTI.UNIGE.IT

The algorithm was drafted and finalized after a discussion between the authors and was then implemented together.

**************************

