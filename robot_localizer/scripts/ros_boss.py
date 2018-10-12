#!/usr/bin/env python
import rospy
import tf
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from robot_localizer.tf_helper import TFHelper
import message_filters

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
    def __init__(self, start=True,
            use_tf=True, sync=True, slop=0.05):
        """
        Args:
            use_tf(bool): Get Odom from tf (default : True)
            sync(bool): Synchronize Odom-Scan Topics (only enabled if use_tf=False, default: True)
            slop(float): Sync slop window (only enabled if sync:=True, default : 0.05)
        """
        self.use_tf_ = use_tf
        self.sync_   = (not use_tf) and sync
        self.slop_   = slop

        # Data
        self.scan_ = None
        self.odom_ = None
        self.part_msg_ = PoseArray()
        self.best_msg_ = PoseStamped()
        self.mark_msg_ = MarkerArray()

        # ROS Handles
        self.part_pub_ = None
        self.mark_pub_ = None
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

    def start(self):
        """ register ROS handles and subscribe to all incoming topics """
        self.part_pub_ = rospy.Publisher('particles', PoseArray, queue_size=5)
        self.mark_pub_ = rospy.Publisher('particles_mk', MarkerArray, queue_size=5)
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
        """ Convert particle to ROS Pose.

        Args:
            p: [3] array formatted (x,y,h)

        Returns:
            msg : geometry_msgs/Pose() equivalent to p
        """
        (x,y,h) = p
        txn = [x,y,0]
        rxn = tf.transformations.quaternion_about_axis(h, [0,0,1])
        msg = self.tfh_.convert_translation_rotation_to_pose(txn,rxn)
        return msg

    def publish(self, particles, best_particle=None,
            weights=None
            ):
        """Publish particles pose / visualization

        Args:
            particles: [N,3] numpy array formatted (x,y,h)
            best_particle : [3] numpy array formatted (x,y,h)

        Returns:
            None
        """
        self.part_msg_.header.frame_id = 'map'
        self.part_msg_.header.seq += 1
        self.part_msg_.header.stamp = rospy.Time.now()


        if best_particle is not None:
            self.best_msg_.header.frame_id = 'map'
            self.best_msg_.header.seq += 1
            self.best_msg_.header.stamp = rospy.Time.now()

        if weights is not None:
            weights = (0.1 / weights.max()) * weights

        # populate poses
        print 'length of particle : ', len(particles)
        self.part_msg_.poses = [self.particle_to_pose(p) for p in particles]
        self.part_pub_.publish(self.part_msg_)

        # poses v2
        self.mark_msg_.markers=[]
        n = len(particles)
        for i in range(n):
            mk = Marker(header=Header(stamp=rospy.Time.now(), frame_id='map'))
            mk.ns = 'pcloud'
            mk.id = i
            mk.type=mk.ARROW
            mk.action=mk.ADD
            mk.pose=self.part_msg_.poses[i]
            if weights is not None:
                s = weights[i]
                mk.scale = Vector3(s*2,s/3.,s/3.)
                mk.color = ColorRGBA(1.0,0.0,1.0,s)
            else:
                mk.scale = Vector3(0.1,0.02,0.02)
                mk.color = ColorRGBA(1.0,0.0,1.0,1.0)
            mk.lifetime = rospy.Duration(1.0) # stay alive for 1 sec

            self.mark_msg_.markers.append(mk)
        self.mark_pub_.publish(self.mark_msg_)

        if best_particle is not None:
            self.best_msg_.pose = self.particle_to_pose(best_particle)
            self.best_pub_.publish(self.best_msg_)

    @property
    def odom(self):
        """ Get latest pose from TF/Cached Msg.

        Returns:
            odom(np.ndarray): None if odom is not available yet;
                otherwise [3] array formatted [x,y,h]
        """
        pose_msg = None
        if self.use_tf_:
            try:
                #pose_tf = self.tfl_.lookupTransform('base_link', 'odom', rospy.Time(0))
                pose_tf = self.tfl_.lookupTransform('odom', 'base_link', rospy.Time(0))
            except tf.Exception as e:
                rospy.loginfo_throttle(1.0, 'Failed TF Transform : {}'.format(e) )
                return None
            pose_msg = self.tfh_.convert_translation_rotation_to_pose(*pose_tf)
        else:
            if self.odom_ is not None:
                pose_msg = self.odom_.pose.pose
                self.odom_ = None # clear msg
        try:
            if pose_msg is not None:
                x,y,h = self.tfh_.convert_pose_to_xy_and_theta(pose_msg)
                return np.asarray([x,y,h])
        except Exception as e:
            rospy.loginfo_throttle(1.0, 'Getting Odom information failed : {}'.format(e))
            return None

    @property
    def scan(self):
        """ Get latest scan data.

        Note:
            assumes scan angle corresponds to (0-2*pi)

        Returns:
            scan(np.ndarray): None is scan is not available yet;
                otherwise [N,2] array formatted [angle,range]
        """
        msg = self.scan_
        if msg is not None:
            self.scan_ = None # clear msg
            angles = anorm(np.linspace(0, 2*np.pi, len(msg.ranges), endpoint=True))
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            mask = (msg.range_min < ranges) & (ranges < msg.range_max)
            return np.stack([angles[mask], ranges[mask]], axis=-1)
        else:
            #rospy.loginfo_throttle(1.0, "No scan msg available")
            return None

    @property
    def data(self):
        """ Get latest data+scan

        Returns:
            odom,scan : refer to RosBoss.odom() and RosBoss.scan().
        """
        return self.odom, self.scan

def main():
    rospy.init_node('ros_boss')

    # roll params
    use_tf = rospy.get_param('~use_tf', False)
    sync   = rospy.get_param('~sync', False)
    slop   = rospy.get_param('~slop', 0.01)

    rospy.loginfo('== Parameters ==')
    rospy.loginfo('use_tf : {}'.format(use_tf))
    rospy.loginfo('sync   : {}'.format(sync))
    rospy.loginfo('slop   : {}'.format(slop))
    rospy.loginfo('================')


    rb = RosBoss(start=True,
            use_tf=use_tf, sync=sync, slop=slop)

    # random particles to test with
    ps = np.random.uniform(size=[10,3])
    bp = np.mean(ps, axis=0)

    rate = rospy.Rate(50)

    last_recv_odom = rospy.Time.now()
    last_recv_scan = rospy.Time.now()

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        odom, scan = rb.data

        # visualize data logs
        if odom is not None:
            last_recv_odom=now
            rospy.loginfo_throttle(1.0, 'Current Position : {}'.format(odom))

        if odom is not None:
            last_recv_scan=now
            rospy.loginfo_throttle(1.0, 'Min Dist : {}'.format(np.min(scan[:,1])))

        # debugging ...
        dt_odom = (now - last_recv_odom).to_sec()
        if dt_odom > 5.0:
            rospy.logerr_throttle(5.0, "Haven't received odom data in {} seconds!".format(dt_odom))

        dt_scan = (now - last_recv_scan).to_sec()
        if dt_scan > 5.0:
            rospy.logerr_throttle(5.0, "Haven't received odom data in {} seconds!".format(dt_scan))

        rb.publish(ps, bp)
        rate.sleep()

if __name__ == "__main__":
    main()
