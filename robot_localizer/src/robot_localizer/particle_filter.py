#!/usr/bin/env python
import numpy as np
import utils as U
from resample import resample
import random

class ParticleFilter(object):
    def __init__(self):
        # general parameters
        self.size_ = None
        self.particles_ = None
        self.weights_ = None
        self.gamma_ = (4.0 / 8.0) # TODO : determine if gamma is useful
        self.best_ = None

        # initialization parameters
        self.radius = 10 #the limit of how big the particle field is generated.
        self.spread = .5 #radius, theta_std

        self.recalc_ = True

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
        if seed:
            delta = self.particles_ - np.reshape(seed, [1,3])
            cost = np.linalg.norm(delta, axis=-1)
            self.weights_ = np.full(size, 1.0/size)
            #self.weights_ = (1.0 / (cost + 1.0/size))
            #self.weights_ /= self.weights_.sum()
        else:
            self.weights_ = np.full(size, 1.0/size)
        self.size_ = size
        self.recalc_ = True

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

        if self.best_ is None:
            self.recalc_ = True
        else:
            # apply transform
            x,y,h = self.best_
            bc, bs = np.cos(h), np.sin(h)
            x += bc*dx - bs*dy
            y += bs*dx + bc*dy
            h = U.anorm(h+dh)
            self.best_ = np.asarray([x,y,h])

    def resample(self, weight, noise=(0.0,0.0,0.0),
            eps=1e-3, inv=False):
        # cost -> probability
        if inv:
            weight = 1.0 / (np.add(weight, eps))

        # weight rectification
        weight[np.isnan(weight)] = 0.0

        # naive choice
        #idx = np.random.choice(self.size_, size=self.size_, p=prob)
        #particles = self.particles_[idx]

        # "perfect" resampler
        #print 'weight statistics', weight.min(), weight.max(), weight.std() #np.min(weight), np.max(weight), np.std(weight)
        weight = (self.gamma_ * self.weights_) + (1.0 - self.gamma_) * weight

        self.particles_, self.weights_ = resample(self.particles_, weight)

        # apply noise (simple method to sample more variance in candidates)
        self.particles_ = np.random.normal(loc=self.particles_, scale=noise)

        self.recalc_ = True # need to recompute best particle

        # "naive" best
        #best_idx = np.argmax(self.weights_)
        #return self.particles_[best_idx].copy()

        # "weighted mean" best
        return self.best
        #x, y = np.average(self.particles[:,:2], axis=0, weights=self.weights_)
        #h = U.amean(self.particles[:,2], w=self.weights_)
        #return [x,y,h]

    @property
    def particles(self):
        return self.particles_
        #return np.copy(self.particles_)

    @property
    def best(self):
        if self.recalc_ and self.particles_ is not None:
            x, y = np.average(self.particles[:,:2], axis=0, weights=self.weights_)
            h = U.amean(self.particles[:,2], w=self.weights_)
            self.best_ = np.asarray([x,y,h])
            self.recalc_ = False
        else:
            return self.best_

def main():
    n = 1000
    pf = ParticleFilter()
    pf.initialize(size=n)

    # visualization
    from matplotlib import pyplot as plt
    ps0 = pf.particles

    cost0 = np.linalg.norm(ps0[:,:2], axis=-1) # test: cost by distance from zero
    kp, kx = 0.5, 1.0 # configure as p=50% match at 1.0m distance
    k = (- np.log(kp) / kx)
    p0 = np.exp(-k*cost0)
    p0 = U.renorm(p0, 0.2, 0.7)

    S = np.reshape
    nax = np.newaxis

    c0 = np.full((n, 3), [1.0,0.0,0.0])
    col0 = np.concatenate([c0, p0[:,nax]], axis=-1)

    pf.resample(p0, noise=(0.5,0.5,0.5))
    ps1 = pf.particles
    cost1 = np.linalg.norm(ps1[:,:2], axis=-1) # test: cost by distance from zero
    p1 = np.exp(-k*cost1)
    p1 = U.renorm(p1, 0.2, 0.7)

    c1 = np.full((n, 3), [0.0,1.0,0.0])
    col1 = np.concatenate([c1, p1[:,nax]], axis=-1)

    sc0 = 20.0 * p0
    sc1 = 20.0 * p1

    plt.scatter(ps0[:,0], ps0[:,1], label='original', c=col0, s=sc0)
    plt.scatter(ps1[:,0], ps1[:,1], label='resample', c=col1, s=sc1)

    plt.xlim(-10.0, 10.0)
    plt.ylim(-10.0, 10.0)

    plt.legend(loc=1)
    ax = plt.gca()
    ax.set_axisbelow(True)
    plt.grid()

    leg = ax.get_legend()
    hl_dict = {handle.get_label(): handle for handle in leg.legendHandles}
    hl_dict['original'].set_color([1.0,0.0,0.0])
    hl_dict['resample'].set_color([0.0,1.0,0.0])
    plt.title('Resample (single step snapshot)')
    plt.show()

    ##pf.particles_ = np.random.uniform(-10, 10, size=(100,2))
    #cost = np.random.uniform(0, 10, size=100)
    #pf.resample(cost)

if __name__ == "__main__":
    main()
