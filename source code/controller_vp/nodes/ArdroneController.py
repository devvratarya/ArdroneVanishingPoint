#!/usr/bin/env python
'''
ArdroneController interacts with Ardorne to send it the final velocities. 
Joystick and Vanishing point velocities are published to goal_vel.
Controller subscribes to this topic and sends the final velocities 
to drone. It runs at 100 hz and updates the velocities.

'''

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
import pid
import time

__author__ = "Devvrat Arya"
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "Devvrat Arya"
__email__ = "devvratarya15@gmail.com"
__status__ = "Production"


class ArdroneController:
    def __init__( self ):
        print "controller startup"
        self.nav_sub = rospy.Subscriber( "ardrone/navdata", Navdata, self.callback_navdata, queue_size=10 )
        
        #publish velocities to drone
        self.cmd_vel_pub = rospy.Publisher( "cmd_vel", Twist, queue_size=10 )
        
        #gets velocities from joystick or vanishing point
        self.goal_vel_sub = rospy.Subscriber( "goal_vel", Twist, self.callback_goal_vel, queue_size=10 )

        # gain_kp, gain_ki, gain_kd initiation
        self.linearxpid = pid.Linear_Pid( 0.5, 0.0, 0.5 )
        self.linearypid = pid.Linear_Pid( 0.5, 0.0, 0.5 )

        #current velocities of drone, recieved from ardrone/navdata topic
        self.vx = self.vy = self.vz = self.ax = self.ay = self.az = 0.0
        self.last_update = None
        self.goal_vel = Twist()

    def callback_goal_vel( self, data ):
        self.goal_vel = data

    def callback_navdata( self, data ):
        
        #getiing speeds from the drone
        self.vx = data.vx/1e3
        self.vy = data.vy/1e3
        self.vz = data.vz/1e3

    def update( self ):
        if self.last_update == None:
            self.last_update = rospy.Time.now()
            dt = 0.0
        else:
            current_update = rospy.Time.now()
            dt = ( current_update - self.last_update ).to_sec()
            self.last_update = current_update
            
        cmd = Twist()
        cmd.angular.y = 0
        cmd.angular.x = 0
        cmd.angular.z = self.goal_vel.angular.z
        cmd.linear.z = self.goal_vel.linear.z

        #linear velocities according to PID
        cmd.linear.x = self.linearxpid.update( self.goal_vel.linear.x, self.vx, 0.0, dt )
        cmd.linear.y = self.linearypid.update( self.goal_vel.linear.y, self.vy, 0.0, dt )
        #print "setting goal point velocities"
        #print cmd.linear.x
        #print cmd.linear.z
        #print cmd.angular.z
        self.cmd_vel_pub.publish( cmd )

def main():
  rospy.init_node( 'ArdroneController' )

  controller = ArdroneController()
  r = rospy.Rate(100)

  try:
      while not rospy.is_shutdown():
          controller.update()
          r.sleep()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
