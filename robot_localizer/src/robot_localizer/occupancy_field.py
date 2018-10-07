""" An implementation of an occupancy field that you can use to implement
    your particle filter """

from __future__ import print_function, division
import time
import numpy as np
import rospy
import cv2
from nav_msgs.srv import GetMap
from sklearn.neighbors import NearestNeighbors

class OccupancyField(object):
    """ Stores an occupancy field for an input map.  An occupancy field returns
        the distance to the closest obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to
            the closest obstacle
    """

    def __init__(self):
        # grab the map from the map server
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map
        self.shape_ =  (self.map.info.width, self.map.info.height)
        self.map_ = np.reshape(self.map.data, self.shape_)

        # unroll useful metadata
        info = self.map.info
        self.ox_ = info.origin.position.x
        self.oy_ = info.origin.position.y
        self.mres_ = info.resolution
        self.mw_ = info.width
        self.mh_ = info.height

        obs = (self.map_ > 0) | (self.map_ < 0) # treat -1 as obstacle
        obs = np.logical_not(obs).astype(np.uint8) # now zero is an obstacle (format for cv2)

        self.dist_ = cv2.distanceTransform(obs, cv2.DIST_L2, 3)
        self.dist_ *= self.mres_

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """

        ix = np.int32((x - self.ox_) / self.mres_)
        iy = np.int32((y - self.oy_) / self.mres_)
        #print(ix, iy)
        d = self.dist_[iy, ix] # WARN : must be indexed as (iy,ix)
        
        x_out = np.logical_or(ix<0, ix>=self.mw_)
        y_out = np.logical_or(iy<0, iy>self.mh_)
        xy_out = np.logical_or(x_out, y_out)
        #print('valid : {}/{}'.format(d.size - np.sum(xy_out), d.size))

        if np.isscalar(d):
            if xy_out:
                d = np.nan
        else:
            d[xy_out] = np.nan
        return d

        ##x_coord = \
        ##    int((x - self.ox_)/self.m)
        ##y_coord = \
        ##    int((y - self.oy_)/self.map.info.resolution)

        ## check if we are in bounds
        #if x_coord > self.map.info.width or x_coord < 0:
        #    return float('nan')
        #if y_coord > self.map.info.height or y_coord < 0:
        #    return float('nan')

        #ind = x_coord + y_coord*self.map.info.width
        #if ind >= self.map.info.width*self.map.info.height or ind < 0:
        #    return float('nan')

        #return self.dist_[x_coord, y_coord]
