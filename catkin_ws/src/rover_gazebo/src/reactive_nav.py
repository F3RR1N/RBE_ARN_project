#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import PointStamped
#from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path


# callback functions to update the sensor range
def callback_IR_front(data):
    global IR_front_prox
    IR_front_prox = data.range
    #print "IR_front = ", data.range

def callback_IR_frontL(data):
    global IR_frontL_prox
    IR_frontL_prox = data.range
    #print "IR_frontL = ", data.range

def callback_IR_frontR(data):
    global IR_frontR_prox
    IR_frontR_prox = data.range
    #print "IR_frontR = ", data.range

def callback_IR_rear(data):
    global IR_rear_prox
    IR_rear_prox = data.range
    #print "IR_rear = ", data.range

def callback_IR_rearL(data):
    global IR_rearL_prox
    IR_rearL_prox = data.range
    #print "IR_rearL = ", data.range

def callback_IR_rearR(data):
    global IR_rearR_prox
    IR_rearR_prox = data.range
    #print "IR_rearR = ", data.range

def callback_IR_sideL(data):
    global IR_sideL_prox
    IR_sideL_prox = data.range
    #print "IR_sideL = ", data.range

def callback_IR_sideR(data):
    global IR_sideR_prox
    IR_sideR_prox = data.range
    #print "IR_sideR = ", data.range



def reactiveNav():

    # Initialize the node, subsribes to topics of interest and
    # publishes the desired results
    rospy.init_node('reactive_nav')
    print 'Initialized Reactive Navigation'

    # subscribe to the 8 IR sensors and read the sensor range to check for
    # possiblity of collision
    subIR_front = rospy.Subscriber('sensor/ir_front', Range, callback_IR_front)

    subIR_frontL = rospy.Subscriber('sensor/ir_frontL', Range, callback_IR_frontL)

    subIR_frontR = rospy.Subscriber('sensor/ir_frontR', Range, callback_IR_frontR)

    subIR_rear = rospy.Subscriber('sensor/ir_rear', Range, callback_IR_rear)

    subIR_rearL = rospy.Subscriber('sensor/ir_rearL', Range, callback_IR_rearL)

    subIR_rearR = rospy.Subscriber('sensor/ir_rearR', Range, callback_IR_rearR)

    subIR_sideL = rospy.Subscriber('sensor/ir_sideL', Range, callback_IR_sideL)

    subIR_sideR = rospy.Subscriber('sensor/ir_sideR', Range, callback_IR_sideR)


def avoidCollision():
    # define globals
    global IR_front_prox
    global IR_frontL_prox
    global IR_frontR_prox
    global IR_rear_prox
    global IR_rearL_prox
    global IR_rearR_prox
    global IR_sideL_prox
    global IR_sideR_prox

    # Initialize
    IR_front_prox = 0.8
    IR_frontL_prox = 0.8
    IR_frontR_prox = 0.8
    IR_rear_prox = 0.8
    IR_rearL_prox = 0.8
    IR_rearR_prox = 0.8
    IR_sideL_prox = 0.8
    IR_sideR_prox = 0.8

    # desired speeds
    goFront = 1
    goReverse = -0.5
    stop = 0
    steerL = 0.5
    steerR = -0.5
    noSteer = 0

    # desired collision threshold
    THRES_MIN = 0.45


    # Collision Avoidance Strategy
    # check for proximity of each sensor to the obstacle
    # Euclidian distance from sensor lies within the collision threshold
    IR_front_mean = (IR_front_prox + IR_frontL_prox + IR_frontR_prox)/3
    IR_sideL_mean = (IR_frontL_prox + IR_sideL_prox + IR_rearL_prox)/3
    IR_sideR_mean = (IR_frontR_prox + IR_sideR_prox + IR_rearR_prox)/3
    IR_rear_mean = (IR_rear_prox + IR_rearL_prox + IR_rearR_prox)/3

    print "IR_front_mean = ", IR_front_mean
    print "IR_sideL_mean = ", IR_sideL_mean
    print "IR_sideR_mean = ", IR_sideR_mean
    print "IR_rear_mean = ", IR_rear_mean



    # set publisher node
    pubVelocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # define a Twist message to publish the velocity
    roverVel = Twist()
    roverVel.linear.x = goFront
    roverVel.linear.y = stop
    roverVel.linear.z = stop
    roverVel.angular.x = noSteer
    roverVel.angular.y = noSteer
    roverVel.angular.z = noSteer

    if (IR_front_mean < THRES_MIN or IR_rear_mean < THRES_MIN):
        roverVel.linear.x = stop
    elif (IR_front_mean > THRES_MIN and IR_sideL_mean > THRES_MIN and IR_sideR_mean > THRES_MIN):
        roverVel.linear.x = goFront
    else:
        rover.linear.x = goFront

    if (IR_sideL_mean < THRES_MIN or IR_sideR_mean < THRES_MIN):
        roverVel.linear.y = noSteer

    # update rover velocity
    pubVelocity.publish(roverVel)

    # stream relative translation, rotation, absolute position
    # <subscribe to odom and publish these topics>
    # <calculate distance moved by robot and publish as translation>

    # if path tracking is ON, stream path followed by the robot
    #pubNavPath = rospy.Publisher('/reactive_nav/path', Float32MultiArray, queue_size = 1)    obstacleCheck()
    rospy.spin()


if __name__ == "__main__":

    # define globals
    global IR_front_prox
    global IR_frontL_prox
    global IR_frontR_prox
    global IR_rear_prox
    global IR_rearL_prox
    global IR_rearR_prox
    global IR_sideL_prox
    global IR_sideR_prox

    try:
    	reactiveNav()
        avoidCollision()
    except rospy.ROSInterruptException:
    	rospy.logerr('Could not start reactive_nav node')
