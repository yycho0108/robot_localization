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
        self.spread = .5 #radius, theta_std

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
        
        if seed:
            x = np.random.normal(loc=init_x, scale=spread[0], size=size)
            y = np.random.normal(loc=init_y, scale=spread[0], size=size)
            #x, y = np.random.normal(loc=[[init_x],[init_y]], scale=spread, size=(2,size))
            #x, y = np.random.normal(loc=[[init_x],[init_y]], scale=spread, size=(2,size))
            h = np.random.normal(loc=init_theta, scale=spread[1], size=size)
        else:
            x, y = np.random.normal(scale=spread[0], size=(2,size))
            h = np.random.uniform(-np.pi, np.pi, size=size)

        h = U.anorm(h)

        self.particles_ = np.stack([x,y,h], axis=-1)
        self.size_ = size

        #for i in range(size):
        #    # TODO : fix distributions, spread, etc.

        #    if seed:
        #        x, y = np.random.normal(loc=[init_x,init_y], scale=[spread[0],spread[0]], size=(2,size))
        #        h = np.random.normal(loc=init_theta, scale=spread[1], size=size)
        #    else:
        #        x, y = np.random.normal(loc=[0,0], scale=spread[0], size=(size,2))
        #        h = np.random.uniform(-np.pi, np.pi, size=size)

        #    ##generate random coordinate in polar, radius from the start.
        #    #ran_dist = random.uniform(0, spread[0])
        #    #if seed:
        #    #    #If there is a seed, then it will tend to center via gaussian.  If not, complete random.
        #    #    ran_dir = random.gauss(init_theta, spread[1])
        #    #else:
        #    #    ran_dir = random.uniform(0, 2*np.pi)

        #    ##convert to cartesian.
        #    #ran_theta = random.uniform(0, 2*np.pi)
        #    #x = np.cos(ran_dir)*ran_dist + init_x
        #    #y = np.sin(ran_dir)*ran_dist + init_y

        #    particle = [x,y,h]
        #    particle_field.append(particle)

        #self.size_ = size
        #self.particles_= np.asarray(particle_field, dtype=np.float32)

    def update(self, odom):
        # assume odom = (v,w,dt)
        # if odom = (dx,dy,dh)
        dx, dy, dh = odom
        # elif odom = (v,w,dt)
        h = self.particles_[:,2]
        #dx = v * np.cos(h) * dt
        #dy = v * np.sin(h) * dt
        #dh = w * dt

        c_ = np.cos(self.particles_[:,2])
        s_ = np.sin(self.particles_[:,2])

        dx_ =  c_*dx - s_*dy
        dy_ =  s_*dx + c_*dy

        self.particles_[:,:2] += np.stack([dx_,dy_], axis=-1)
        self.particles_[:,2] = U.anorm(self.particles_[:,2] + dh)

    def resample(self, weight, noise=(0.0,0.0,0.0),
            eps=1e-3, viz=False, inv=False):
        # cost -> probability

        if viz:
            plt.figure()
            plt.bar(range(self.size_), weight)
            plt.title('weight')

        if inv:
            prob = 1.0 / (np.add(weight, eps))
        else:
            prob = weight

        prob[np.isnan(prob)] = 0.0
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
        print 'prob', np.min(prob), np.max(prob), np.std(prob)
        self.particles_, ws = resample(self.particles_, prob)
        #self.particles_ += np.random.normal(loc=0.0, scale=noise)
        self.particles_ = np.random.normal(loc=self.particles_, scale=noise)

        # TODO : better "best" particle

        x,y = np.mean(self.particles_[:,:2], axis=0)
        h = U.amean(self.particles[:,2])
        return [x,y,h]

        #return self.particles_[np.argmax(ws)]

    @property
    def particles(self):
        return self.particles_
        #return np.copy(self.particles_)

def main():
    pf = ParticleFilter()
    pf.initialize(size=100)

    # visualization
    from matplotlib import pyplot as plt
    ps0 = pf.particles
    cost0 = np.linalg.norm(ps0[:,:2], axis=-1) # test: cost by distance from zero

    pf.resample(cost0, noise=(0.1,0.1,0.1), inv=True)
    ps1 = pf.particles
    cost1 = np.linalg.norm(ps1[:,:2], axis=-1) # test: cost by distance from zero

    plt.scatter(ps1[:,0], ps1[:,1], label='resample', s=5.0*(cost0.max() / cost1), alpha=1.0)
    plt.scatter(ps0[:,0], ps0[:,1], label='original', s=5.0*(cost1.max() / cost0), alpha=0.5)
    plt.legend()
    plt.show()

    ##pf.particles_ = np.random.uniform(-10, 10, size=(100,2))
    #cost = np.random.uniform(0, 10, size=100)
    #pf.resample(cost)

if __name__ == "__main__":
    main()
