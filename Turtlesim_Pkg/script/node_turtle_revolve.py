#!/usr/bin/env python

# importing all required modules
import rospy
from geometry_msgs.msg import Twist
import sys
import math as mt

# defining a function
def rotate_turtle(lin_vel,ang_vel):    
   
   # establishing the nodes 
   rospy.init_node('rotate_turtle',anonymous=True)
   pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
   vel=Twist()

   #rate
   rate=rospy.Rate(10)

   # As pi is an irrational no. so to complete one revolution the value is taken accordingly to reach the target.
   #converting speed into radians
   angular_speed=ang_vel*(2.08*(mt.pi)/360)
   #converting angle into radians()
   relative_angle= 2.08*(mt.pi)
   
   #Assigning the values to the various component of velocity
   vel.linear.x=lin_vel
   vel.linear.y=0
   vel.linear.z=0

   vel.angular.x =0
   vel.angular.y =0
   vel.angular.z =angular_speed
 
   
   #if rospy is not shutdown then execute the loop
   while not rospy.is_shutdown():
        
        #Setting the current time for distance calculation
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        #Loop to move the turtle in an specified distance
        while(current_angle<relative_angle):
             #Publish the velocity
             pub.publish(vel)

             #Takes actual time to velocity calculation
             t1=rospy.Time.now().to_sec()

             #calculate actual distance covered at the instant of time
             current_angle=vel.angular.z*(t1-t0)


             rospy.loginfo('current_angle=%f t1= %f: relative_angle=%f ',current_angle,t1,relative_angle)
             print("Turtle is moving in a circle")

        #Turle made one revolution and came out of the loop 
        #After the loop, stops the robot
        vel.linear.x = 0
        #Setting current time
        t2 = rospy.Time.now().to_sec()
        #setting current angle
        current_angle2 = 0
        #Desired angle in radians to spin the turtle after completing its one revolution
        relative_angle2=mt.radians(73)
        

        #Loop to move the turtle by desired angle
        while(current_angle2 < relative_angle2):
             #Publish the velocity
             pub.publish(vel)

             #Takes actual time to velocity calculation
             t3 = rospy.Time.now().to_sec()

             #calculate actual distance covered at the instant of time
             current_angle2 = angular_speed*(t3-t2)

        #After the loop, stops the robot from spinning
        vel.angular.z = 0

        #Publish the velocity
        pub.publish(vel)
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        rotate_turtle(1,35)
    except rospy.ROSInterruptException:
        pass
