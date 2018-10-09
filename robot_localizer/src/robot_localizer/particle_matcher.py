#!/usr/bin/env python

from occupancy_field import OccupancyField
import numpy as np

class ParticleMatcher(object):
    #map is a 2D array passed with a singular data value on whether or not there is
    #something in the coordinate
    def __init__(self):
        #self.map_ = map_
        self.OF = OccupancyField()


    def match(self, particle_list, scan):
        #scan is a list of [angle, theta]
        min_dist = np.min(scan[:,1])
        #print('ps', particle_list)
        #dist = [self.OF.get_closest_obstacle_distance(p[0], p[1]) for p in particle_list]
        ps = particle_list
        dist = self.OF.get_closest_obstacle_distance(ps[:,0], ps[:,1])
        dist = np.asarray(dist, dtype=np.float32)
        #print('dist stats : min {} max {} std {}'.format(dist.min(),dist.max(),dist.std()))

        cost = np.abs(np.subtract(dist, min_dist))
        cost[np.isnan(cost)] = 0 # set nan cost to zero to prevent artifacts

        weight = cost.max() - cost + 1e-3
        #weight = 1.0 / cost
        # TODO : determine inverse vs. linear cost performance comparison

        weight[np.isnan(dist)] = 0 # set nan weight to zero to make sure it doesn't get sampled
        #print('ws', weight)
        return weight

        #weight_list = []
        #for i in range(len(particle_list)):
        #    #get angle in degrees for index of scan.
        #    min_dist = np.min(scan[:,1])

        #    dist = self.OF.get_closest_obstacle_distance(particle_list[i][0], particle_list[i][1])
        #    cost = np.abs(min_dist - dist)
        #    #get weight
        #    weight = 1 / (cost + 1e-1)

        #    weight_list.append(weight)

        #return weight_list
