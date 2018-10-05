#!/usr/bin/env python
import numpy as np
import utils as U
from resample import resample
from matplotlib import pyplot as plt
import random

class ParticleFilter(object):
    def __init__(self):
        # general parameters
        self.size_ = None
        self.particles_ = None

        # initialization parameters
        self.radius = 10 #the limit of how big the particle field is generated.
        self.theta_standard_deviation = .5 #Deviation from theta in radians. Maxes at pi.

    def initialize(self, size, seed=None, spread=[10.0,0.5]): #seed = initial pose.
        """ Initializes particles.

        Note:
            The size_ and particles_ property is modified internally every time initialize is called.
            Each generated particle is an array composed of three elements: x,y, theta. Theta is in radians.

        Args:
            size(int): number of particles to generate.
            seed(list): an array of [x,y,h] that indicates initial pose; (0,0,0) if None. (default:None)
            spread(list): an array of (radius, theta_std).
                radius(float): maximum radius of particle generation in a uniform distribution.
                theta_std(float): Standard deviation of the heading in a normal distribution.

        Returns:
            None (internally modified)
        
        """
        init_x = 0.0
        init_y = 0.0
        init_theta = 0.0

        if seed:
            init_x = seed[0]
            init_y = seed[1]
            init_theta = seed[2]

        particle_field = []

        for i in range(size):
            #generate random coordinate in polar, radius from the start.
            ran_dist = random.uniform(0, spread[0])
            ran_theta = random.gauss(init_theta, spread[1])
            #convert to cartesian.
            x = np.cos(ran_theta)*ran_dist + init_x
            y = np.sin(ran_theta)*ran_dist + init_y

            particle = [x,y,ran_theta]
            particle_field.append(particle)

        self.size_ = size
        self.particles_= np.asarray(particle_field, dtype=np.float32)

    def update(self, odom):
        # assume odom = (v,w,dt)
        # if odom = (dx,dy,dh)
        # dx, dy, dh = odom
        # elif odom = (v,w,dt)
        h = self.particles_[:,2]
        dx = v * np.cos(h) * dt
        dy = v * np.sin(h) * dt
        dh = w * dt

        self.particles_ += [[dx,dy,dh]]
        self.particles_[:,2] = U.anorm(self.particles_[:,2])

    def resample(self, cost, noise=(0.0,0.0,0.0),
            eps=1e-3, viz=False):
        # cost -> probability

        if viz:
            plt.figure()
            plt.bar(range(self.size_), cost)
            plt.title('cost')

        prob = 1.0 / (np.add(cost, eps))
        prob /= np.sum(prob)

        if viz:
            plt.figure()
            plt.bar(range(self.size_), prob)
            plt.title('probability')

        # naive choice
        #idx = np.random.choice(self.size_, size=self.size_, p=prob)
        #if viz:
        #    plt.figure()
        #    plt.bar(range(self.size_), np.bincount(idx, minlength=self.size_))
        #    plt.title('selection')
        #if viz:
        #    plt.show()
        #particles = self.particles_[idx]

        # "perfect" resampler
        self.particles_, _ = resample(self.particles_, prob)
        self.particles_ = np.random.normal(self.particles_, scale=noise)

    @property
    def particles(self):
        return np.copy(self.particles_)

def main():
    pf = ParticleFilter()
    pf.initialize(size=100)

    # visualization
    from matplotlib import pyplot as plt
    ps0 = pf.particles
    cost0 = np.linalg.norm(ps0[:,:2], axis=-1) # test: cost by distance from zero

    pf.resample(cost0, noise=(0.1,0.1,0.1))
    ps1 = pf.particles
    cost1 = np.linalg.norm(ps1[:,:2], axis=-1) # test: cost by distance from zero

    plt.scatter(ps1[:,0], ps1[:,1], label='resample', s=50.0*(1.0 / cost1), alpha=1.0)
    plt.scatter(ps0[:,0], ps0[:,1], label='original', s=50.0*(1.0 / cost0), alpha=0.5)
    plt.legend()
    plt.show()

    ##pf.particles_ = np.random.uniform(-10, 10, size=(100,2))
    #cost = np.random.uniform(0, 10, size=100)
    #pf.resample(cost)

if __name__ == "__main__":
    main()
