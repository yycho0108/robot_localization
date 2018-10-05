#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from robot_localizer.tf_helper import TFHelper
from tf import transformations as tx

from occupancy_field import OccupancyField

from ros_boss import RosBoss
from robot_localizer import utils as U
from robot_localizer.particle_filter import ParticleFilter
from robot_localizer.particle_matcher import ParticleMatcher
from std_srvs.srv import Empty, EmptyResponse

from matplotlib import pyplot as plt

class ParticleFilterROS(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # create all compute handles
        self.rb_ = RosBoss()
        self.pf_ = ParticleFilter()
        self.pm_ = ParticleMatcher()

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        self.pose0_sub = rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        self.reset_srv = rospy.Service('reset', Empty, self.reset_cb)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        #self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.last_odom_ = None

    def reset_cb(self, _):
        self.pf_.initialize(
                size=1000, # TODO : configure size
                seed=None,
                spread=[30.0,0.5]
                ) # TODO : configure spread
        return EmptyResponse()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        
        print('what the fuck?', xy_theta)

        self.pf_.initialize(
                size = 1000,
                seed = xy_theta,
                spread = [3.0, 6.28]
                )

        ps = self.pf_.particles

        #plt.scatter(ps[:,0], ps[:,1])
        #plt.show()

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def step(self):
        # in the main loop all we do is continuously broadcast the latest
        # map to odom transform

        if self.pf_.particles is None:
            # no initial pose, return
            self.transform_helper.send_last_map_to_odom_transform()
            return

        odom, scan = self.rb_.data

        if odom is not None:
            if self.last_odom_ is None:
                self.last_odom_ = odom
            delta = odom - self.last_odom_
            delta[2] = U.anorm(delta[2])
            self.pf_.update(delta)
            self.last_odom_ = odom

        best = None
        if scan is not None:
            ps = self.pf_.particles
            ws = self.pm_.match(self.pf_.particles, scan)
            #print('ws', ws)
            #plt.scatter(ps[:,0], ps[:,1], label='resample', s=ws, alpha=1.0)
            #plt.show()
            best = self.pf_.resample(ws)

        self.rb_.publish(self.pf_.particles, best)

        if best is not None:
            x,y,h = best
            t = [x,y,0]
            q = tx.quaternion_about_axis(h, (0,0,1))
            best_pose = self.transform_helper.convert_translation_rotation_to_pose(t,q)
            # TODO : not rigorous timestamp
            self.transform_helper.fix_map_to_odom_transform(best_pose, rospy.Time.now())

        self.transform_helper.send_last_map_to_odom_transform()


    def run(self):
        r = rospy.Rate(5)
        while not(rospy.is_shutdown()):
            self.step()
            r.sleep()

if __name__ == '__main__':
    n = ParticleFilterROS()
    n.run()
