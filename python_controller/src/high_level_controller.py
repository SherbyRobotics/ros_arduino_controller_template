#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


#########################################
class python_controller(object):

    #######################################    
    def __init__(self):

        # Init subscribers  
        self.sub_joy        = rospy.Subscriber("joy", Joy , self.read_joy , queue_size=1)
        self.sub_sensors    = rospy.Subscriber("sensors", Float32MultiArray , self.read_feedback_from_arduino, queue_size=1)

        # Init publishers
        self.pub_cmd    = rospy.Publisher("cmd", Twist , queue_size=1)
        
        # Timer
        self.dt         = 0.05
        self.timer      = rospy.Timer( rospy.Duration( self.dt ), self.timed_controller )
        
        #################
        # Paramters
        #################
        
        # Controller

        #################
        # Memory
        #################
        
        # References Inputs
        self.user_ref        = 0
        self.high_level_mode = 0  # Control mode of this controller node
        
        # Ouput commands
        self.arduino_cmd    = 0  
        self.arduino_mode   = 0  
        
        # Sensings inputs
        self.sensor         = 0  # sensor feeback buffer
        
        # For DEBUG
        #self.tick = 0

        
    #######################################
    def timed_controller(self, timer):
             
        if (self.high_level_mode == 0 ):
            # Full stop mode
            self.arduino_cmd    = 0  
            self.arduino_mode   = 0
            
            #Debug test sinus ref
            #self.tick = self.tick + 1
            #self.arduino_cmd    = 5 * np.sin( 0.01 * self.tick / self.dt )
            
        else:
            ##########################
            # Controllers HERE            
            ##########################
            if  ( self.high_level_mode == 1 ):
                # Open-Loop
                self.arduino_cmd    = self.user_ref
                self.arduino_mode   = 1 
                
            elif ( self.high_level_mode == 2 ):
                # Closed-loop in python node
                self.arduino_cmd    = 10 * ( self.user_ref - self.sensor )
                self.arduino_mode   = 2   
                
            elif ( self.high_level_mode == 3 ):
                pass
                
            elif ( self.high_level_mode == 4 ):
                pass 
                
            elif ( self.high_level_mode == 5 ):
                pass
                
            elif ( self.high_level_mode == 6 ):
                pass
            
        self.send_cmd_to_arduino()


    ####################################### 
    def read_joy( self, joy_msg ):
        """ """
    
        self.user_input        = joy_msg.axes[3]    # Up-down Right joystick 
        self.high_level_mode   = 0            
                
        # Software deadman switch
        #If left button is active 
        if (joy_msg.buttons[4]):
            
            #If right button is active       
            if (joy_msg.buttons[5]):   
                
                self.high_level_mode   = 2
                
            #If button A is active 
            elif(joy_msg.buttons[1]):   
                
                self.high_level_mode   = 3
                
            #If button B is active 
            elif(joy_msg.buttons[2]):   
                
                self.high_level_mode   = 4
                
            #If button x is active 
            elif(joy_msg.buttons[0]):   
                
                self.high_level_mode   = 5
                
            #If button y is active 
            elif(joy_msg.buttons[3]):   
                
                self.high_level_mode   = 6
                
            #If left trigger is active 
            elif (joy_msg.buttons[6]):
                
                self.high_level_mode   = 7
                
            #If bottom arrow is active
            elif(joy_msg.axes[5]):
                
                self.high_level_mode   = 8
            
            # No active button
            else:
                self.high_level_mode   = 1
        
        # Deadman is un-pressed
        else:
            
            self.user_input        = 0
            self.high_level_mode   = 0
      
      
    ##########################################################################################
    def send_cmd_to_arduino(self):
 
      #Init encd_info msg
      cmd_prop = Twist()
      
      #Msg
      cmd_prop.linear.x  = self.arduino_cmd   # Command 
      cmd_prop.linear.z  = self.arduino_mode     # Arduino mode

      # Publish cmd msg
      self.pub_cmd.publish(cmd_prop)


    ####################################### 
    def read_feedback_from_arduino( self, msg):

        # Read feedback from arduino
        self.sensor       = msg.data[0]
        

#########################################

if __name__ == '__main__':
    
    rospy.init_node('controller',anonymous=False)
    node = python_controller()
    rospy.spin()
