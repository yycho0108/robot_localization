#!/usr/bin/env python

from occupancy_field import OccupancyField

class ParticleMatcher(object):
    #map is a 2D array passed with a singular data value on whether or not there is
    #something in the coordinate
    def __init__(self, map_):
        self.map_ = map_
        self.OF = OccupancyField()


    def match(self, particle_list, scan):
        #scan is a list of [angle, theta]
        weight_list = []
        for i in len(particle_list):
            #get angle in degrees for index of scan.
            min_dist = np.min(scan[:,1])

            dist = self.OF.get_closest_obstacle_distance(particle_list[i][0], particle_list[i][1])
            #get weight
            weight = 1 / np.absolute((min_dist - dist))

            weight_list.append(weight)

        return weight_list
