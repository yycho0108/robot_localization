import rospy
import tf
import numpy as no

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped
from robot_localizer.tf_helper import TFHelper

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
    def __init__(self, use_tf=True):
        """
        Args:
            use_tf(bool): Get Odom from tf (default : True)
        """
        self.use_tf_ = use_tf

        # Data
        self.scan_msg_ = None
        self.scan_ = None 
        self.pose_ = None

        # ROS Handles
        self.scan_sub_ = None
        self.odom_sub_ = None
        self.tfl_ = None
        self.tfh_ = None

        self.subscribe()

    def scan_cb(self, msg):
        self.scan_msg_ = msg

    def scan_proc(self):
        """ Process Scan Data.
        Note:
            assumes scan angle corresponds to (0-2*pi)
        """
        angles = U.anorm(np.linspace(0, 2*np.pi, len(msg.ranges), endpoint=True))
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        mask = (msg.range_min < ranges) & (ranges < msg.range_max)
        self.scan_ = np.stack([angles[mask], ranges[mask]], axis=-1)

    def subscribe(self):
        """ subscribe to all incoming topics """
        self.scan_sub_ = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        if not self.use_tf_:
            self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.tfl_ = tf.TransformListener()
        self.tfh_ = TFHelper()

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
        pass

    def get_odom(self):
        """ Get Pose from TF """
        if self.use_tf_:
            try:
                pose_tf = self.tfl_.lookupTransform('base_link', 'odom', rospy.Time(0))
            except tf.Exception as e:
                rospy.loginfo_throttle(1.0, 'Failed TF Transform : {}'.format(e) )
                return None
            pose_msg = self.tfh_.convert_translation_rotation_to_pose(*pose_tf)
        else:
            pose_msg = self.pose_
        try:
            x,y,h = self.tfh_.convert_pose_to_xy_and_theta(pose_msg)
            return np.asarray([x,y,h])
        except Exception as e:
            rospy.loginfo_throttle(1.0, 'Odom information not available : {}'.format(e))
            return None

    def get_scan(self):
        """ Get latest scan """
        return self.scan_
