#!/usr/bin/env python

# importing modules

import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, sqrt, atan2
import numpy as np
import tf
import math

velocity_msg =Twist()
pid_theta=None
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose=[0,0,0]
yaw_ = 0
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    1: 'turn left',
    2: 'follow the wall',
    0: 'go to point',
}


desired_position_x = 12.5
desired_position_y = 0
desired_position_z = 0

yaw_precision_ = math.pi / 90
dist_precision_ = 0.3

def Waypoints(t):
	x=[1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6.2]
	y=[0.80684536,1.3598625,1.5302948,1.1358817,0.281533,-0.69033153,-1.3763171,-1.52118,-1.1477789,-0.53855444,-0.0069098248]
	return[x[t],y[t]]


def control_loop():
    global pid_theta
    global velocity_msg
    global pose

    rospy.init_node('ebot_controller', anonymous=False)
    rospy.loginfo("Press CTRL + C to terminate")


    pose[0] = 0.0
    pose[1] = 0.0
    pose[2] = 0.0
    pid_theta = PID(0,0,0)
    rate = rospy.Rate(10)
    
    

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    sub = rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    
    

 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)





    while not rospy.is_shutdown():
    	
    	

     for i in range(11):
        
            move_to_point(Waypoints(i)[0],Waypoints(i)[1])
            rospy.sleep(1)
        
    
     
     
     main()
     rospy.logwarn("Action done.")
     print("Controller message pushed at {}".format(rospy.get_time()))
     velocity_msg.linear.x = 0
     velocity_msg.angular.z = 0
     pub.publish(velocity_msg)
    
     rospy.sleep(1)
      


def move_to_point(x, y):

        linear=0

        diff_x = x - pose[0]
        diff_y = y - pose[1]
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  
        theta = atan2(diff_y, diff_x)
        
        
        pid_theta.setPID(1, 0, 0)     
        pid_theta.setPoint(theta)
        



        
        while not rospy.is_shutdown():

            angle = pid_theta.update(pose[2])
            
            if abs(angle) > 0.20:
                angle =  1.5*angle/abs(angle)*0.2

            if abs(angle) <= 0.1:
                break

            velocity_msg.linear.x = 0
    	    velocity_msg.angular.z = angle
            pub.publish(velocity_msg)
            rospy.sleep(1)

       
        

        pid_theta.setPoint(theta)
        pid_theta.setPID(1, 0.02, 0.2) 







        while not rospy.is_shutdown():
            diff_x = x - pose[0]
            diff_y = y - pose[1]
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) 


            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2
            angle = pid_theta.update(pose[2])

            if abs(angle) > 0.2:
                angular = angular/abs(angle)*0.2
                
            if abs(linear) < 0.15 and abs(angle) < 0.1:
                break
            
            
            velocity_msg.linear.x = linear
    	    velocity_msg.angular.z = angle
            pub.publish(velocity_msg)
            rospy.sleep(1)
        done()

        

class PID:
    
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:  
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

    	
def odom_callback(data):
    global pose
    
    counter = 0
    trajectory = list() 

    counter += 1
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, tf.transformations.euler_from_quaternion([x,y,z,w])[2]]

def laser_callback(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    
def change_state_new(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def fix_yaw(x,y):
    global regions_
    regions = regions_
    d = 1.5
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(y - pose[1], x - pose[0])
    err_yaw = desired_yaw - pose[2]
    velocity_msg = Twist()

    if math.fabs(err_yaw) > yaw_precision_ and err_yaw > 0:
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.2
        velocity_msg.angular.z = 0.43    
        pub.publish(velocity_msg)

    elif math.fabs(err_yaw) > yaw_precision_ and err_yaw <0:
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.2
        velocity_msg.angular.z = -0.43    
        pub.publish(velocity_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_ and regions['fright'] > d:
        print 'Yaw error: [%s]' % err_yaw
        change_state_new(1)

def go_straight_ahead(x,y):
    global regions_
    regions = regions_
    d = 1.5
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(y - pose[1], x - pose[0])
    err_yaw = desired_yaw - pose[2]
    err_pos = math.sqrt(pow(y - pose[1], 2) + pow(x - pose[0], 2))
    
    if err_pos > dist_precision_ and regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.8
        pub.publish(velocity_msg)

    elif err_pos > dist_precision_ and  regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:   
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.8
        pub.publish(velocity_msg)
        pass

    elif err_pos > dist_precision_ and regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.8
        pub.publish(velocity_msg) 
        pass   

    elif err_pos > dist_precision_ and regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        velocity_msg = Twist()
        velocity_msg.angular.z = 0.43
        pub.publish(velocity_msg)
        pass

    elif err_pos > dist_precision_ and regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
         twist_msg = Twist()
         twist_msg.angular.z = 0.43
         pub.publish(twist_msg)
         pass

    elif err_pos > dist_precision_ and regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
         twist_msg = Twist()
         twist_msg.angular.z = 0.43
         pub.publish(twist_msg)
         pass

    elif err_pos > dist_precision_ and regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
         velocity_msg = Twist()
         velocity_msg.angular.z = 0.43 
         pub.publish(velocity_msg)
         pass

    elif err_pos > dist_precision_ and regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
         velocity_msg = Twist()
         velocity_msg.linear.x = 0.85
         velocity_msg.angular.z = -0.45
         pub.publish(velocity_msg)

    else:
        print 'Position error: [%s]' % err_pos
        change_state_new(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_ and regions['front'] > d and regions['fright'] > d and regions['fleft'] > d:
        print 'Yaw error: [%s]' % err_yaw
        change_state_new(0)


def done():
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)   

def main():
    global desired_position_x
    global desired_position_y
    desired_position_x = 12.5
    desired_position_y = 0
    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(desired_position_x,desired_position_y)
        elif state_ == 1:
            go_straight_ahead(desired_position_x,desired_position_y)
        elif state_ == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        #rate.sleep()


    
       

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass





