#!/usr/bin/env python

'''
ArDroneFollower serves two objectives. Get the joystick 
velocities and transfer them in to drone velocities. And 
it gets the vanishing point and calcules the yaw angle 
i.e. angular.z. ArDroneFollower send valocities are recieved 
by Controller which sends the final velocities to the drone. 
'''


import roslib,rospy
import sys
import math
import pid
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Image
from ardrone_autonomy.msg import Navdata
from ardrone_autonomy.srv import LedAnim
import std_srvs.srv

__author__ = "Devvrat Arya"
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "Devvrat Arya"
__email__ = "devvratarya15@gmail.com"
__status__ = "Production"


# These value are defined to set the joystick key controls. These are snet via launch
# and assigned in the constructor
# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 0
ButtonLand      = 1
ButtonTakeoff   = 2

# define the default mapping between joystick axes and their corresponding directions
AxisRoll        = 4
AxisPitch       = 5
AxisYaw         = 2
AxisZ           = 3

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0



class ArDroneFollower:
    def __init__( self ):
        print "waiting for driver to startup"
        rospy.wait_for_service( "ardrone/setledanimation")
        print "driver started"

        self.ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
        self.ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
        self.ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
        self.AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
        self.AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
        self.AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
        self.AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
        self.ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
        self.ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
        self.ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
        self.ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )
        self.autocmd = 8

        self.led_service = rospy.ServiceProxy( "ardrone/setledanimation", LedAnim )

        self.vp_sub = rospy.Subscriber( "ardrone_vp/vanishing_point",
                                             Point, self.vanishing_point_cb , queue_size=10)
        self.goal_vel_pub = rospy.Publisher( "goal_vel", Twist, queue_size=10 )
        self.vp_found_time = None

        self.dorne_img_sub = rospy.Subscriber( "ardrone/front/image_raw", Image, self.image_cb)

        #This is a function that periodically calls a callback.
        self.timer = rospy.Timer( rospy.Duration( 0.50 ), self.timer_cb, False )

        self.land_pub = rospy.Publisher( "ardrone/land", Empty )
        self.takeoff_pub = rospy.Publisher( "ardrone/takeoff", Empty )
        self.reset_pub = rospy.Publisher( "ardrone/reset", Empty )

        self.yaw_max = 0.5
        self.roll_max = 1.0
        self.pitch_max = 2.0

        #Increasing the P term for yaw
        self.xPid = pid.VP_Pid( 0.040, 0.0, 0.0, self.yaw_max )
        self.yPid = pid.VP_Pid( 0.050, 0.0, 0.0, self.pitch_max )
        self.zPid = pid.VP_Pid( 0.050, 0.0, 0.0, self.roll_max )

        # alpha defines the proportion of last frame to be considered
        self.alpha = 0.5

        self.xPid.setPointMin = 45
        self.xPid.setPointMax = 55

        self.yPid.setPointMin = 40
        self.yPid.setPointMax = 60

        self.zPid.setPointMin = 32
        self.zPid.setPointMax = 40

        self.lastAnim = -1

        self.noVP = False
        self.noVPCounter = 0
        self.validPoint = 0

        self.vp_found = Point( 0, 0, -1 )
        self.isPoint = False
        self.old_cmd = self.current_cmd = Twist()

        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy, queue_size=10 )
        self.manual_cmd = False
        self.auto_cmd = False

        self.bridge = CvBridge()

        self.navdata_sub = rospy.Subscriber( "/ardrone/navdata", Navdata, self.navdata_cb, queue_size=10 )
        self.navdata = None
        
        #standard states from AR Drone
        self.states = { 0: 'Unknown', 
                        1: 'Init',
                        2: 'Landed',
                        3: 'Flying',
                        4: 'Hovering',
                        5: 'Test',
                        6: 'Taking Off',
                        7: 'Goto Fix Point',
                        8: 'Landing',
                        9: 'Looping' }
        
    def navdata_cb( self, data ):
        self.navdata = data

    
    def image_cb( self, data ):
        try:
            #passthrough remains the encodingvof the original image
            cv_image = self.bridge.imgmsg_to_cv( data, "passthrough" )
        except CvBridgeError, e:
            print e
        
        self.drone_image = np.asarray( cv_image )
    
        
    def callback_joy( self, data ):
        empty_msg = Empty()

        #print "joystick interrupted"

        if(data.buttons[self.ButtonEmergency] == 1):
            print "Emergency"
        if(data.buttons[self.ButtonLand] == 1):
            print "Land"
        if(data.buttons[self.ButtonTakeoff] == 1):
            print "Takeoff"

        if(data.buttons[9] == 1):
            print "Initiated"
        
        
        if data.buttons[self.ButtonTakeoff] == 1 and self.last_joy_cmd[self.ButtonTakeoff] == 0:
            self.takeoff()

        if data.buttons[self.ButtonLand] == 1 and self.last_joy_cmd[self.ButtonLand] == 0:
            self.land()

        if data.buttons[self.ButtonEmergency] == 1 and self.last_joy_cmd[self.ButtonEmergency] == 0:
            self.reset()
           
        if data.buttons[self.autocmd] == 1 and self.last_joy_cmd[self.autocmd] == 0:
            #print self.auto_cmd
            self.auto_cmd = not self.auto_cmd
            if self.auto_cmd == True:
                print "auto control enabled"
            else:
                print "mannual control enabled"
            

        #with out setting last joy cmd, the joystick controller will not work
        self.last_joy_cmd = data.buttons

        self.current_cmd = Twist()

        self.current_cmd.angular.x = self.current_cmd.angular.y = 0
        #data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw,data.axes[AxisZ]/ScaleZ
        self.current_cmd.angular.z = data.axes[AxisYaw] * self.angularZlimit

        self.current_cmd.linear.z = data.axes[AxisZ] * self.linearZlimit
        self.current_cmd.linear.y = data.axes[AxisRoll] * self.linearXlimit
        self.current_cmd.linear.x = data.axes[AxisPitch] * self.linearXlimit

        #if no velocity is issued by joystick, set mannual command to false
        if ( self.current_cmd.linear.x == 0 and
             self.current_cmd.linear.y == 0 and
             self.current_cmd.linear.z == 0 and
             self.current_cmd.angular.z == 0 ):
            self.manual_cmd = False
            #print "mannual command false"
        else:
            self.setLedAnim( 9 )
            print "mannual control enabled"
            self.manual_cmd = True
            #print "mannual command true"

        self.goal_vel_pub.publish( self.current_cmd )
        

    def setLedAnim( self, animType, freq = 10 ):
        if self.lastAnim == type:
            return

        msg = LedAnim();
        msg.type = animType;
        msg.freq = freq;
        msg.duration = 3600;
        
        self.led_service( type = animType, freq = freq, duration = 255 )
        self.lastAnim = type

    def takeoff( self ):
        self.takeoff_pub.publish( Empty() )
        self.setLedAnim( 9 )

    def land( self ):
        self.land_pub.publish( Empty() )

    def reset( self ):
        self.reset_pub.publish( Empty() )

    def vanishing_point_cb( self, data ):
        #print "found point"
        if data.z != -1.0:
            if self.vp_found.z == -1.0:
                self.vp_found = data
            else:
                # update the ema vp_found, if we didn't just re-acquire
                self.vp_found.x = self.vp_found.x * self.alpha + data.x * ( 1.0 - self.alpha )
                self.vp_found.y = self.vp_found.y * self.alpha + data.y * ( 1.0 - self.alpha )
                self.vp_found.z = self.vp_found.z * self.alpha + data.z * ( 1.0 - self.alpha )
            #print self.vp_found.x, self.vp_found.y, self.vp_found.z
            #print data.z
            #print self.vp_found.z
        else:
            self.vp_found = data

        self.vp_found_time = rospy.Time.now()
        self.isPoint = True

    def hover( self ):
        hoverCmd = Twist()
        self.goal_vel_pub.publish( hoverCmd )

    def hover_cmd_cb( self, data ):
        self.hover()


    def timer_cb( self, event ):
        
        #print "checking"
        # If no point, set vp_vp_found as
        # (0,0,-1)
        if ( self.vp_found_time == None or ( rospy.Time.now() - self.vp_found_time ).to_sec() > 1 ):
            self.vp_found = Point( 0, 0, -1.0 )
            self.vp_found_time = rospy.Time.now()
        if event.last_real == None:
            dt = 0
        else:
            dt = ( event.current_real - event.last_real ).to_sec()
        '''
        self.isPoint is set to true when VP is found, 
        the velocoties based on vp are updated only then.
        this keeps that cmd and vp in sync
        '''
        if self.isPoint == True:
            
            self.current_cmd = Twist()

            '''
            if self.vp_found.z == -1.0:
                self.noVPCounter = self.noVPCounter + 1
                self.validPoint = 0
            else:
                self.validPoint = self.validPoint + 1 
                self.noVPCounter = 0
            '''    
            if self.vp_found.z == -1.0:
            #if self.noVPCounter > 3:
                self.current_cmd.angular.z = 0.5
                #self.current_cmd = Twist()
                self.setLedAnim( 0, 2 )           
            else:
                #self.noVP = False
                #self.validPoint = 0
                self.current_cmd.angular.z = self.xPid.get_output( self.vp_found.x,dt )
                self.current_cmd.linear.z = 0
                self.current_cmd.linear.x = self.zPid.get_output( self.vp_found.z, dt )
                self.setLedAnim( 8, 2 )

                
            if self.auto_cmd == False or self.manual_cmd == True:
                self.setLedAnim( 9 )
                return
            

            self.goal_vel_pub.publish( self.current_cmd )
            #self.vp_found = Point( 0, 0, -1 )
            self.isPoint = False


def main():
    rospy.init_node( 'ArdroneFollower' )
    print "starting"
    follower = ArDroneFollower()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Keyboard interrupted"

if __name__ == '__main__':
    main()
