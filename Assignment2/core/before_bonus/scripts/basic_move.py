#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from math import cos, sin, asin, tan, atan2
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Range
from std_srvs.srv import Empty
from random import random
import numpy  as np
from PID import PID

# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def from_twist_to_np(twist):
    linear = twist.linear
    angular = twist.angular

    return np.array([
        [linear.x, linear.y, linear.z],
        [angular.x, angular.y, angular.z]])

def from_pos_to_np(pos):
    position = pos.position
    # angular = pos.angular

    return np.array([position.x, position.y, position.z])
    
class Params:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class Hook:
    def __init__(self):
        pass

    def on_update_pose(self, thymio):
        pass

    def on_receive_sensor_data(self,thymio,data, sensor_id, name):
        pass

class EigthWriter(Hook):
    def __init__(self, linear_vel=0.1, angular_vel=0.5, tol=10**(-3)):
        self.initial_pose = None
        self.has_move = False
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.tol = tol
        self.has_finished = False
        self.count = 0

    def on_update_pose(self, thymio):
        if thymio == None: return

        if self.has_finished: 
            thymio.vel_msg.linear.x = 0
            thymio.vel_msg.angular.z = 0
            return
        
        if self.initial_pose == None: self.initial_pose = from_pos_to_np(thymio.current_pose)

        current_pose = thymio.current_pose
        pose = from_pos_to_np(current_pose)

        thymio.vel_msg.linear.x = self.linear_vel
        thymio.vel_msg.angular.z = self.angular_vel

        err = np.mean(np.abs(self.initial_pose - pose))
        is_far_from_start = err > self.tol 

        self.has_move = self.has_move or is_far_from_start

        if self.has_move and not is_far_from_start:
            self.initial_pose = pose # reset pose to calibrate
            self.angular_vel  = -1 * self.angular_vel
            self.count += 1
        
        self.has_finished =  self.count == 2


class RandomMovementHook:
    def on_update_pose(self, thymio):
        thymio.vel_msg.linear.x = random()

class GoCloseTurnAndGoForward(Hook):

    def __init__(self):
        self.stable = False
        self.approach_wall = True
        self.should_turn = False
        self.forward = False

    def done_on_align(self):
        self.thymio.vel_msg.angular.z = 0.0
        self.thymio.vel_msg.linear.x = 0.0
        self.approach_wall = False
        self.should_turn = True 
        print('aligned')
        self.thymio.start = self.thymio.time_elapsed

    def not_touch_align(self):
        self.thymio.vel_msg.linear.x = 0.1

    def has_touched_align(self):
        self.thymio.vel_msg.linear.x = 0.0
        self.thymio.vel_msg.angular.z = 0.0
        print('touched')
        
    def turn_until(self, data,name,  name1, name2, not_touch, has_touch, done):
        if name != name1 and name != name2: return 
        has_touched = np.abs(data.max_range - data.range) > 0.02
        e = np.abs(self.thymio.sensors_cache[name1].range -  self.thymio.sensors_cache[name2].range)
        print  e
        self.stable = e < 0.015
        if not has_touched:
            not_touch()
        if self.stable and has_touched:
            done()
        if has_touched and not self.stable:
            has_touch()
            step = self.thymio.pid.step(e, 0.01)
            if name == name1:
                if  data.range < self.thymio.sensors_cache[name2].range:
                    self.thymio.vel_msg.angular.z = step
            elif name == name2:
                if  data.range < self.thymio.sensors_cache[name1].range:
                    self.thymio.vel_msg.angular.z = -step

    def on_receive_sensor_data(self,thymio,data, sensor_id, name):
        self.thymio = thymio

        if self.approach_wall:
            self.turn_until(data,name,'left','right',self.not_touch_align, self.has_touched_align, self.done_on_align)
        elif self.should_turn:
            if thymio.time_elapsed - thymio.start <  32 * 10 * 2 * 2:
                thymio.vel_msg.angular.z = 0.5
            else: 
                thymio.vel_msg.angular.z = 0
                self.forward = True
                self.should_turn = False
                thymio.start = thymio.time_elapsed 
        elif self.forward:
            # multiply by 9 since the rear sensors are more or less 10 cm
            if thymio.time_elapsed - thymio.start <  9 * 10 *  10 * 2 * 2:
                thymio.vel_msg.linear.x = 0.1
            else: 
                self.forward = False
                thymio.vel_msg.linear.x = 0



def callback(fun,*args, **kwargs):
    def delay(x):
        fun(x, *args, **kwargs)
    return delay

class BasicThymio:

    def __init__(self, thymio_name, hooks=[]):
        """init"""
        self.thymio_name = thymio_name
        self.hooks = hooks
        self.sensors_cache = {}

        self.time_elapsed = 0
        self.start = 0

        rospy.init_node('basic_thymio_controller', anonymous=True)

        # Publish to the topic '/thymioX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(self.thymio_name + '/odom',
                                                Odometry, self.update_state)
        self.sensors_names = ['rear_left','rear_right','left','center_left','center', 'center_right' ,'right']

        self.sensors_subscribers = [rospy.Subscriber(self.thymio_name + '/proximity/' + sensor_name, 
        Range, 
        callback(self.sensors_callback,i,sensor_name)) for i,sensor_name in enumerate(self.sensors_names)]

        self.current_pose = Pose()
        self.current_twist = Twist()
        # publish at this rate
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()
        self.pid = PID(2,1,0.5)


    def sensors_callback_bang_bang(self, data, sensor_id, name):
        self.sensors_cache[name] = data
        try:
            has_touched = data.max_range != data.range
            e = np.abs(self.sensors_cache['left'].range -  self.sensors_cache['right'].range)
            print e
            self.stable = e < 0.025

            if self.stable:
                self.vel_msg.angular.z = 0.0
            if has_touched and not self.stable:
                if name == 'left':
                    if  data.range < self.sensors_cache['right'].range:
                        self.vel_msg.angular.z = 0.1
                        print('turn right')
                    print data.range, name
                elif name == 'right':
                    if  data.range < self.sensors_cache['left'].range:
                        self.vel_msg.angular.z = -0.1
                        print('turn left')
                    print data.range, name
        except KeyError:
            # The cache may not be full
            pass

    def sensors_callback(self, data, sensor_id, name):
        self.sensors_cache[name] = data

        try:
            for hook in self.hooks:
                hook.on_receive_sensor_data(thymio, data, sensor_id, name)
        except KeyError:
            pass

    def thymio_state_service_request(self, position, orientation):
        """Request the service (set thymio state values) exposed by
        the simulated thymio. A teleportation tool, by default in gazebo world frame.
        Be aware, this does not mean a reset (e.g. odometry values)."""
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.thymio_name
            model_state.reference_frame = '' # the frame for the pose information
            model_state.pose.position.x = position[0]
            model_state.pose.position.y = position[1]
            model_state.pose.position.z = position[2]
            qto = quaternion_from_euler(orientation[0], orientation[0], orientation[0], axes='sxyz')
            model_state.pose.orientation.x = qto[0]
            model_state.pose.orientation.y = qto[1]
            model_state.pose.orientation.z = qto[2]
            model_state.pose.orientation.w = qto[3]
            # a Twist can also be set but not recomended to do it in a service
            gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = gms(model_state)

            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def update_state(self, data):
        """A new Odometry message has arrived. See Odometry msg definition."""
        # Note: Odmetry message also provides covariance
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion (quat)

        # rospy.loginfo("State from Odom: (%.5f, %.5f, %.5f) " % (self.current_pose.position.x, self.current_pose.position.y, yaw))
        self.time_elapsed += 10
        for hook in self.hooks:
            hook.on_update_pose(self)

    def update_vel(self, linear, angular):
        vel_msg = Twist()

        vel_msg.linear.x = linear.x 
        vel_msg.linear.y = linear.y 
        vel_msg.linear.z = linear.z

        vel_msg.angular.x = angular.x 
        vel_msg.angular.y = angular.y 
        vel_msg.angular.z = angular.z

        self.vel_msg = vel_msg

    def move(self, linear, angular):
        """Moves the migthy thymio"""
        self.update_vel(linear, angular)

        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()

        rospy.spin()

    def basic_move(self):
        """Moves the migthy thymio"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.2 # m/s
        vel_msg.angular.z = 0. # rad/s

        while not rospy.is_shutdown():
            # Publishing thymo vel_msg
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            self.rate.sleep()

        # Stop thymio. With is_shutdown condition we do not reach this point.
        #vel_msg.linear.x = 0.
        #vel_msg.angular.z = 0.
        #self.velocity_publisher.publish(vel_msg)

        # waiting until shutdown flag (e.g. ctrl+c)
        rospy.spin()

def usage():
    return "Wrong number of parameters. basic_move.py [thymio_name]"

if __name__ == '__main__':
    if len(sys.argv) == 2:
        thymio_name = sys.argv[1]
        print "Now working with robot: %s" % thymio_name
    else:
        print usage()
        sys.exit(1)
    # ex1 
    # thymio = BasicThymio(thymio_name, hooks=[EigthWriter()])
    # Teleport the robot to a certain pose. If pose is different to the
    # origin of the world, you must account for a transformation between
    # odom and gazebo world frames.
    # NOTE: The goal of this step is *only* to show the available
    # tools. The launch file process should take care of initializing
    # the simulation and spawning the respective models
    
    # thymio.thymio_state_service_request([0.,0.,0.], [0.,0.,0.])
    # rospy.sleep(1.)

    # thymio.move(Params(0), Params(0))

    # ex2
    thymio = BasicThymio(thymio_name, hooks=[GoCloseTurnAndGoForward()])
    thymio.thymio_state_service_request([4,0,0.], [0.0,0.0,0])
    thymio.move(Params(0), Params(0))
    rospy.sleep(1.)

    # thymio.move(Params(0.2), Params(0))


