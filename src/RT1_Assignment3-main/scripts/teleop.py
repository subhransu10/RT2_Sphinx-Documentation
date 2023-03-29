#!/usr/bin/env python3

"""
.. module::teleop
    :platform: Unix
    :synopsis: Python module for Autonomous driving of the robot
.. 

Subscribes to:
    /mode to recover the current modality
    /scan to obtain informations.for example - calculate the obstacle distance that is detected by the laser scanner

Publishes to:
    /mode to change the modality whenever the user wants to quit

This particluar script carries out both *assisted* and *without assistance* driving.
The main portion of the script that I worked on was linked to the Assisted Driving modality because the script is essentially a rewrite of *teleop twist keyboard* since it already allows the robot to move using keyboard.
It subscribes to the _/scan_ topic to determine whether a certain direction is clear of obstacles or not, lets say a wall.
The robot cannot avoid an obstacle if it is travelling backward because it can only see through its lasers within a +-90 relative degree range.
By pressing 'p' on the teleop console, or alternatively by pushing another key, the user can "exit" both modalities.
"""
from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import os



from sensor_msgs.msg import LaserScan #for mode 3
import time
from std_srvs.srv import *
from std_msgs.msg import Int32


msg1 = """
- - -  WITHOUT ASSISTANCE  - - -
- - - MANUAL DRIVING - - -
--------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

PRESS 'p' TO QUIT THE MODE AND RETURN IN 'IDLE' STATUS

"""

moveBindings_mode2 = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
        
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }


#Mode 3  

msg2 = """
- - - -  ASSISTED  - - - -
- - - MANUAL DRIVING - - -
--------------------------
Moving around:
       i    
   j   k    l

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

PRESS 'p' TO QUIT THE MODE AND RETURN IN 'IDLE' STATUS

"""

can_go_left = True
can_go_right = True
can_go_straight = True
firstTime=False
currentmode=0
quit=False

moveBindings_mode3 = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(-1,0,0,0),
        'p':(0,0,0,0),
    }
    
    

class colors:
    """
    Class used to print colors on the terminal
    """
    PINK = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m' 
    RED = '\033[91m'
    ORANGE = '\033[33m' 
    PURPLE  = '\033[35m'

    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def my_stop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        # Publish.
        self.publisher.publish(twist)

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    """
    This function prints the current linear and angular speed

    Args:
        speed(double): current speed
        turn(double): current angular speed
    Returns:
        (str):string that contains the linear and angular speed

    """
    return "currently:\tspeed %s\tturn %s " % (speed,turn)



def clbk_laser(msg): 
    """
    Callback function which is called to recover information about the obstacle distance
    Args:
        msg(sensor_msgs/LaserScan.msg): contains ranges[] array which provides the distances each laser and the objects in the environment

    """

    global can_go_left
    global can_go_right
    global can_go_straight

    #retrieving the minimum frontal and lateral distances, if the distance is greater or equal to 1 meter, then that distance will be set to 1
    right = min(min(msg.ranges[0:143]), 0.8)
    front = min(min(msg.ranges[288:431]), 0.8)
    left = min(min(msg.ranges[576:719]), 0.8)

    #if the value is not '1', it means that it is lower and therefore the robot should not be able to go towards that direction
    if right != 0.8:
        can_go_right =False
    else:
        can_go_right =True

    if front != 0.8:
        can_go_straight =False
    else:
        can_go_straight =True

    if left != 0.8:
        can_go_left =False
    else:
        can_go_left =True



def routine(dictionary,msg,assisted):
            """
            Function that binds the pressed key to the actual motion of the robot.

            Args:
                dictionary(dict): a dictionary that contains all the selectable keys
                msg(str): string containing the instructions for the selected modality
            
            """
            global key,x,y,z,th,status,speed,turn,pub_thread,firstTime,quit
            def disable_commands(dictionary):
                """
                Function for assisted driving. It manages the allowed commands the user can impose. The allowed commands depends on the free directions in the proximity of the robot

                Args:
                    dictionary(dict): the dictionary that has to be filtered by popping the keys that are binded to the unallowed directions
                
                """
                global can_go_left
                global can_go_right
                global can_go_straight
                

                #checking all possible combinations of the allowed directions in order to pop from the dictionary the unallowed ones

                if not can_go_straight and not can_go_right and not can_go_left:
                    dictionary.pop('i') #can't go straight
                    dictionary.pop('j') #can't go left
                    dictionary.pop('l') #can't go right

                elif not can_go_left and not can_go_straight and can_go_right:
                    dictionary.pop('i')
                    dictionary.pop('j')
                    print(colors.RED +colors.BOLD + "CAN'T GO LEFT AND STRAIGHT     "+colors.ENDC, end = "\r")

                elif can_go_left and not can_go_straight and not can_go_right:
                    dictionary.pop('i')
                    dictionary.pop('l')
                    print(colors.RED +colors.BOLD+"CAN'T GO RIGHT AND STRAIGHT      "+colors.ENDC, end = "\r")
             
                elif not can_go_left and can_go_straight and not can_go_right :
                    dictionary.pop('l')
                    dictionary.pop('j')
                    print(colors.RED +colors.BOLD+"CAN'T GO LEFT AND RIGHT          "+colors.ENDC, end = "\r")

                elif can_go_left and not can_go_straight and can_go_right:

                    dictionary.pop('i')
                    print(colors.RED +colors.BOLD+"CAN'T GO STRAIGHT                "+colors.ENDC, end = "\r")

                elif not can_go_left and can_go_straight and can_go_right :
                    dictionary.pop('j')
                    print(colors.RED +colors.BOLD+"CAN'T GO LEFT                    "+colors.ENDC, end = "\r")

                elif can_go_left and can_go_straight and not can_go_right:
                    dictionary.pop('l')
                    print(colors.RED +colors.BOLD+"CAN'T GO RIGHT                   "+colors.ENDC, end = "\r")
                else:
                    print(colors.GREEN +colors.BOLD+"All movements are allowed =)   "+colors.ENDC, end = "\r")


            key = getKey(key_timeout)

            if(assisted):
                disable_commands(dictionary)
            if key in dictionary.keys():
                x = dictionary[key][0]
                y = dictionary[key][1]
                z = dictionary[key][2]
                th = dictionary[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                

            pub_thread.update(x, y, z, th, speed, turn)

            if(key=='p'):
                    os.system('cls||clear')
                    firstTime=True
                    quit=True

def mode_callback(data):
    """
    Callback function to set the local variable of the current mode if it has been changed by a node

    Args:
        data(int): An integer that represents the current modality
    """
    global currentmode
    currentmode=data.data




def main():
    """
    Main function: creates the *PublishThread* object and manages the modality that needs to be runned accordingly to the choice of the user.
    """
    global key,x,y,z,th,status,speed,turn,pub_thread,firstTime,quit,key_timeout,settings
    rospy.init_node('teleop')
    settings = termios.tcgetattr(sys.stdin)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser) #subscription to /scan topic
    pubModality=rospy.Publisher('mode',Int32,queue_size=10)
    subModality=rospy.Subscriber('mode', Int32,mode_callback)
    
    if key_timeout == 0.0:
        key_timeout = None
	
    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    moveBindings_temp = {}    
    pub_thread.wait_for_subscribers()
    pub_thread.update(x, y, z, th, speed, turn)

    mode_type = 0 #by default the mode_type is set to 0 (0: idle, 1:not assisted driving, 2: assisted  driving)
    firstTime=True

    while(1):
        if quit==True:
            pubModality.publish(0)
            quit=False
        if currentmode == 2: 
            if mode_type==0 or mode_type==2: #clear the console and print the proper message if the mode has been changed
                os.system('cls||clear')
                print(msg1)
                mode_type = 1
            
            routine(moveBindings_mode2,msg1,False)

            
             
        elif currentmode==3:
            if mode_type==0 or mode_type==1: #clear the console and print the proper message if the mode has been changed
                os.system('cls||clear')
                print(msg2)
                mode_type = 2
            
            moveBindings_temp = moveBindings_mode3.copy() #instead of using the normal moveBindings dictionary, let's use the one without the unallowed directions
            routine(moveBindings_temp,msg2,True)
            
        else:
            if not firstTime and mode_type == 1 or mode_type==2:
                pub_thread.my_stop()
                os.system('cls||clear')
                print(colors.BLUE +  colors.BOLD + "TELEOP TWIST! Select from the 'user_interface' console which mode you want to run:\n - [2] MANUAL DRIVE, \n - [3] ASSISTED DRIVE"+colors.ENDC)
            if(firstTime):
                firstTime=False
                print(colors.BLUE +  colors.BOLD + "TELEOP TWIST! Select from the 'user_interface' console which mode you want to run:\n - [2] MANUAL DRIVE, \n - [3] ASSISTED DRIVE"+colors.ENDC)
            mode_type = 0



if __name__=="__main__":
    main()
