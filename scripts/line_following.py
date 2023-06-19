#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32, Int32, Bool, String
from geometry_msgs.msg import Twist, Pose2D

class puzzlebot:
    def __init__(self): 

        # Initialize ROS publishers for robot speed and pose, and subscribers for wheel speeds and goal pose
        self.main_pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 10)
        self.pub = rospy.Publisher("position", Pose2D, queue_size = 10)
        self.pub2 = rospy.Publisher("error_theta", Float32, queue_size = 10)
        self.pub3 = rospy.Publisher("error_d", Float32, queue_size = 10)
        self.data_out = Pose2D()
        self.data_out.x = 0.0
        self.data_out.y = 0.0
        self.data_out.theta = 0.0
        rospy.Subscriber("/wr", Float32, self.callbackWr)
        rospy.Subscriber("/wl", Float32, self.callbackWl)
        rospy.Subscriber("/alignment", Int32, self.callbackAlignment)
        rospy.Subscriber("/light_color", String, self.callbackColor)
        rospy.Subscriber("/sign", String, self.callbackSign)
        rospy.Subscriber("/line", Bool, self.callbackLine)
        self.start_time = rospy.get_time()
        print("Position node is running")

        # Initialize variables for wheel speeds, robot pose, target pose, and error values
        self.wr = 0.0
        self.wl = 0.0
        self.r = 0.05
        self.l = 0.19
        self.x_k = 0.0
        self.y_k = 0.0
        self.theta_k = 0.0
        self.error_d = 0.0
        self.error_theta = 0.0
        self.last_error_theta = 0.0
        self.last_error_follow = 0.0
        self.last_error_d = 0.0
        self.trajectory = []
        self.target_x = 0.
        self.target_y = 0.
        self.last_target = [0,0]

        # Initialize Twist message for robot speed
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0

        # Initialize flags and parameters for control logic
        self.beginning = False
        self.current_step = 0 
        self.check = 0
        self.pause = 0
        self.hold = False
        self.angular_threshold_reached = False

        # Initialize PID controller gains and integrals
        self.kp_follow = 0.0015
        self.ki_follow = 0.00
        self.kd_follow = 0.00005
        self.kp_angular = 0.48
        self.ki_angular = 0.0001
        self.kp_linear = 0.02
        self.integral_angular = 0.0
        self.integral_follow = 0.0

        # Variables for input storing
        self.light_color = "n"
        self.sign = ""
        self.line_exist = False
        self.line_error = 0

        # Current state of the state machine and the desired linear speed
        self.state = "line_following"
        self.desired_speed = 0.0

    
    def wrapTheta(self, angle):
        #Keep the input angle between values of -pi and pi
        if(angle > np.pi or angle <= -np.pi):
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

    def PID_Angular(self):
        #PID controller for angular velocity
        self.integral_angular += self.error_theta * self.dt
        output = self.kp_angular * self.error_theta + self.ki_angular * self.integral_angular
        self.last_error_theta = self.error_theta
    
        return output
    
    def PID_Linear(self):
        #PID controller for linear velocity
        max_lineal = 0.07
        output = self.kp_linear * self.error_d
        self.last_error_d = self.error_d
        if(output > max_lineal): output = max_lineal
        return output
    
    def PID_Following(self):
        #PID controller for angular velocity
        self.integral_follow+= self.line_error * self.dt
        derivative = (self.line_error - self.last_error_theta)
        output = self.kp_follow * self.line_error + self.ki_follow * self.integral_follow + self.kd_follow * derivative
        self.last_error_follow = self.line_error
        
        return output

    def callbackWr(self, msg):
        self.wr = msg.data
    
    def callbackWl(self, msg):
        self.wl = msg.data
    
    def callbackAlignment(self,msg):
        self.line_error = -0.5*msg.data

    def callbackColor(self,msg):
        self.light_color = msg.data

    def callbackSign(self,msg):
        self.sign = msg.data

    def callbackLine(self, msg):
        self.line_exist = msg.data

    def stop(self):
        #On exit stop the puzzlebot
        self.speed.angular.z = 0.0
        self.speed.linear.x = 0.0
        self.main_pub.publish(self.speed)
        print("Stopping")
    
    def followingLine(self):
        # Following line state considering traffic light

        if(self.light_color == "g"): actual_speed = self.desired_speed
        elif(self.light_color == "y"): actual_speed = self.desired_speed/2
        elif(self.light_color == "r"): actual_speed = 0
        else: actual_speed = self.desired_speed
        self.speed.linear.x = actual_speed

        if(abs(self.line_error) > 15 and self.sign_inst_completed): self.speed.angular.z = self.PID_Following()
        else: self.speed.angular.z = 0.0

        
        print("Following the line")

    def workInProgress(self):
        # Reduce velocity for work in progress sign state
        
        if(self.pause < 500):
            self.speed.linear.x = self.desired_speed//2

            if(abs(self.line_error) > 15 and self.sign_inst_completed): self.speed.angular.z = self.PID_Following()
            else: self.speed.angular.z = 0.0
            self.pause += 1
        else:
            self.pause = 0
            self.state = "line_following"
        
        print("Going through work in progress")

    def stopSign(self):
        # State to briefly halt when in front of a stop sign
        
        if(self.pause < 500):
            self.speed.linear.x = 0
            self.speed.angular.z = 0.0
            self.pause += 1
        else: 
            self.pause = 0
            self.state = "line_following"
        
        print("Stop sign ahead")
    
    def goForward(self):
        # Accelerate when seeing a go forward sign
        
        if(self.pause < 500):
            self.speed.linear.x = self.desired_speed * 2
            self.speed.angular.z = 0.0
            self.pause += 1
        else:
            self.pause = 0 
            self.state = "line_following"
        
        print("Going straight")

    def turn(self, direction):
        # Turn the robot using position estimation depending on the direction input
        
        if(direction == "left"):
            self.target_x = 0.3
            self.target_y = 0.5
        elif(direction == "right"):
            self.target_x = 0.3
            self.target_y = -0.5

        # Calculate error of the actual position to the desired target and adjust accordingly
        self.error_theta = (np.arctan2(self.target_y - self.y_k, self.target_x - self.x_k)) - self.theta_k
        self.error_d = np.sqrt((self.target_x-self.x_k)**2 + (self.target_y-self.y_k)**2)
        self.error_theta = self.wrapTheta(self.error_theta)

        if(self.pause < 440):
            self.speed.linear.x = self.desired_speed
            self.speed.angular.z = 0.
            self.theta_k = 0.
            self.x_k = 0.
            self.y_k = 0.
            self.pause +=1

        elif(np.rad2deg(abs(self.error_theta)) > 3.0 ):
            self.speed.linear.x = 0.015
            self.speed.angular.z = self.PID_Angular()
        else:
            self.pause = 0
            self.state = "line_following"

        print("I'm turning to the " + str(direction)) 



    def run(self):

        self.dt = rospy.get_time() - self.start_time

        if(self.beginning == False):
            print("Initializing")
        else:
            if(self.hold == True):
                rospy.sleep(0.1)
                self.hold = False

            # State Machine
            
            if(self.state == "line_following"):

                self.followingLine()

                if((self.sign == "right" or self.sign == "left") and not self.line_exist and self.light_color != "r"):
                    self.state = "turn"
                elif(self.sign == "forward" and not self.line_exist and self.light_color != "r"):
                    self.state = "forward"
                elif(self.sign == "work"):
                    self.state = "work"
                elif(self.sign == "stop"):
                    self.state = "stop"

            elif(self.state == "turn"):

                self.turn(self.sign)

            elif(self.state == "forward"):
                self.goForward()
            
            elif(self.state == "work"):
                self.workInProgress()

            elif(self.state == "stop"):
                self.stopSign()

            else:
                self.state = "line_following" 
            

             # Publish robot's speed
            self.main_pub.publish(self.speed)
            self.start_time = rospy.get_time()

        if(self.check > 2 ):
            self.beginning = True
        self.check += 1


if __name__ == '__main__':
    rospy.init_node("line_following")
    
    robot = puzzlebot()
    rate = rospy.Rate(100)
    rospy.on_shutdown(robot.stop)
    robot.desired_speed = 0.05

    try:
        while not rospy.is_shutdown():
            robot.run()
            rate.sleep()
    except rospy.ROSInterruptException():
        pass
