#!/usr/bin/env python
import rospy
import tf
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from robot_localizer.tf_helper import TFHelper

def anorm(x):
    # TODO : replace with U.anorm when utils.py becomes available through PR
    return (x+np.pi) % (2*np.pi) - np.pi

class RosBoss(object):
    """Handles ROS I/O communications and conversions.

    Note:
        Although this class handles most ROS-related code,
        all nodes using this class will need to manually call rospy.init_node()
        to prevent possible conflicts in complex-node scenarios.

    Attributes:
        scan_ (np.ndarray): [N,2] Filtered array of (h,r)
        pose_ (np.ndarray, optional): [3]  Latest pose, (x,y,h)
            note that pose_ is only supported when use_tf:=False.
    """
    def __init__(self, use_tf=True, start=True, sync=True, slop=0.01):
        """
        Args:
            use_tf(bool): Get Odom from tf (default : True)
            sync(bool): Synchronize Odom-Scan Topics (only enabled if use_tf=False)
        """
        self.use_tf_ = use_tf
        self.sync_   = (not use_tf) and sync
        self.slop_   = slop

        # Data
        self.scan_ = None
        self.pose_ = None
        self.part_msg_ = PoseArray()
        self.best_msg_ = PoseStamped()

        # ROS Handles
        self.part_pub_ = None
        self.best_pub_ = None
        self.scan_sub_ = None
        self.odom_sub_ = None
        self.sync_sub_ = None # for synchronized version
        self.tfl_ = None
        self.tfh_ = None

        if start:
            self.start()

    def scan_cb(self, msg):
        """ store scan msg """
        self.scan_= msg

    def odom_cb(self, msg):
        """ store odom msg """
        self.odom_ = msg

    def data_cb(self, scan_msg, odom_msg):
        """ store synced scan/odom msg """
        self.scan_cb(scan_msg)
        self.odom_cb(odom_msg)

    def scan_proc(self):
        """ Process Scan Data.

        Note:
            assumes scan angle corresponds to (0-2*pi)
        """
        msg = self.scan_
        if msg is not None:
            self.scan_ = None # clear msg
            angles = anorm(np.linspace(0, 2*np.pi, len(msg.ranges), endpoint=True))
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            mask = (msg.range_min < ranges) & (ranges < msg.range_max)
            return np.stack([angles[mask], ranges[mask]], axis=-1)
        else:
            rospy.loginfo_throttle(1.0, "No scan msg available")
            return None

    def start(self):
        """ register ROS handles and subscribe to all incoming topics """
        self.part_pub_ = rospy.Publisher('particles', PoseArray, queue_size=5)
        self.best_pub_ = rospy.Publisher('best_particle', PoseStamped, queue_size=5)
        if self.sync_:
            scan_sub = message_filters.Subscriber('scan', LaserScan)
            odom_sub = message_filters.Subscriber('odom', Odometry) 
            self.sync_sub_ = message_filters.ApproximateTimeSynchronizer(
                    [scan_sub, odom_sub], 10, self.slop_, allow_headerless=True)
            self.sync_sub_.registerCallback(self.data_cb)
        else:
            self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)
            if not self.use_tf_:
                self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.tfl_ = tf.TransformListener()
        self.tfh_ = TFHelper()

    def particle_to_pose(self, p):
        (x,y,h) = p
        txn = [x,y,0]
        rxn = tf.transformations.quaternion_about_axis(h, [0,0,1])
        msg = self.tfh_.convert_translation_rotation_to_pose(txn,rxn)
        return msg

    def publish(self, particles, best_particle):
        """Publish particles pose / visualization

        Note:
            Do not include the `self` parameter in the ``Args`` section.

        Args:
            particles: [N,3] numpy array formatted (x,y,h)
            best_particle : [3] numpy array formatted (x,y,h)

        Returns:
            None
        """
        self.part_msg_.header.frame_id = 'map'
        self.part_msg_.header.seq += 1
        self.part_msg_.header.stamp = rospy.Time.now()

        self.best_msg_.header.frame_id = 'map'
        self.best_msg_.header.seq += 1
        self.best_msg_.header.stamp = rospy.Time.now()

        # populate poses
        self.part_msg_.poses = [self.particle_to_pose(p) for p in particles]
        self.part_pub_.publish(self.part_msg_)

        self.best_msg_.pose = self.particle_to_pose(best_particle)
        self.best_pub_.publish(self.best_msg_)

        # TODO : publish best particle

    def get_odom(self):
        """ Get Pose from TF/Cached Msg """
        if self.use_tf_:
            try:
                pose_tf = self.tfl_.lookupTransform('base_link', 'odom', rospy.Time(0))
            except tf.Exception as e:
                rospy.loginfo_throttle(1.0, 'Failed TF Transform : {}'.format(e) )
                return None
            pose_msg = self.tfh_.convert_translation_rotation_to_pose(*pose_tf)
        else:
            pose_msg = self.pose_
            self.pose_ = None # clear msg
        try:
            x,y,h = self.tfh_.convert_pose_to_xy_and_theta(pose_msg)
            return np.asarray([x,y,h])
        except Exception as e:
            rospy.loginfo_throttle(1.0, 'Odom information not available yet : {}'.format(e))
            return None

    def get_scan(self):
        """ Get latest scan """
        return self.scan_proc()

    def get_data(self):
        return self.get_odom(), self.get_scan()

def main():
    rospy.init_node('ros_boss')
    rb = RosBoss(start=True)

    ps = np.random.uniform(size=[10,3])
    bp = np.mean(ps, axis=0)

    while not rospy.is_shutdown():
        odom, scan = rb.get_data() 
        if odom is not None and scan is not None:
            print odom, scan.shape
        rb.publish(ps, bp)

if __name__ == "__main__":
    main()
