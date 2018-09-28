#!/usr/bin/env python
import numpy as np
import utils as U

from matplotlib import pyplot as plt

class Resampler(object):
    def __init__(self):
        pass

    def randomize(self, n):
        u = np.empty(n, dtype=np.float32)
        #mu_0 = (i_0 / T) #??
        # mu???
        mu_0 = (1.0 - mu ** (1.0 / (n+1)))
        u[0] = (1 - mu) ** (1.0 / n)
        for i in range(1,n):
            u[i] = u[i-1] + (1 - u[i-1])(1 - mu) ** (1.0 / (n-i+1))
        return u

    def merge(self, s, w, u):
        j = 0
        t = u[0]
        s2 = np.empty_like(s)
        for i in range(n):
            mu = u[i]
            while mu < t:
                t += w[j]
                j += 1
            s2[i] = s[j]
        return s2

    def __call__(self, s, w):
        u = self.randomize()
        return self.merge(s,w,u)

class ParticleFilter(object):
    def __init__(self):
        self.size_ = 100
        self.particles_ = None

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

    def resample(self, cost, eps=1e-3, copy=False, viz=False):
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
        idx = np.random.choice(self.size_, size=self.size_, p=prob)
        if viz:
            plt.figure()
            plt.bar(range(self.size_), np.bincount(idx, minlength=self.size_))
            plt.title('selection')
        if viz:
            plt.show()
        particles = self.particles_[idx]
        if copy:
            return np.copy(particles)
        else:
            return particles

def main():
    pf = ParticleFilter()
    pf.particles_ = np.random.uniform(-10, 10, size=(100,2))
    cost = np.random.uniform(0, 10, size=100)
    pf.resample(cost)

if __name__ == "__main__":
    main()
