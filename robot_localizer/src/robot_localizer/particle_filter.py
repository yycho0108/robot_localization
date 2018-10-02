#!/usr/bin/env python
import numpy as np
import random

class ParticleFilter(object):
    def __init__(self):
        self.radius = 10 #the limit of how big the particle field is generated.
        self.theta_standard_deviation = .5 #Deviation from theta in radians.  Maxes at pi.
        self.particle_field = None


    def initialize(self, seed, num): #seed = initial pose.
        #Each particle is an array composed of three elements: x,y, theta.  Theta is in radians.
        init_x = seed[0]
        init_y = seed[1]
        init_theta = seed[2]

        particle_field = []

        for i in range(num):
            #generate random coordinate in polar, radius from the start.
            ran_dist = random.uniform(0, self.radius)
            ran_theta = random.gauss(init_theta, self.theta_standard_deviation)
            #convert to cartesian.
            x = np.cos(ran_theta)*ran_dist + init_x
            y = np.sin(ran_theta)*ran_dist + init_y

            particle = [x,y,ran_theta]
            particle_field.append(particle)

        self.particle_field = particle_field
