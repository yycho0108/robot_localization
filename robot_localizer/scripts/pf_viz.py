#!/usr/bin/env python

import numpy as np
from robot_localizer.particle_filter import ParticleFilter
from robot_localizer import utils as U
from matplotlib import pyplot as plt
from matplotlib import animation

class PFAnim(object):
    """
    An animated scatter plot using matplotlib.animations.FuncAnimation.
    adapted from https://stackoverflow.com/questions/9401658/how-to-animate-a-scatter-plot
    """
    def __init__(self, pf, n_iter=50):
        self.pf_ = pf

        # Setup the figure and axes...
        self.fig, self.ax = plt.subplots()
        # Then setup FuncAnimation.
        self.ani = animation.FuncAnimation(self.fig, self.update, n_iter, interval=100, 
                                           init_func=self.setup_plot, blit=True)

    def pprob(self):
        ps = self.pf_.particles
        cost = np.linalg.norm(ps[:,:2], axis=-1)
        kp, kx = 0.5, 1.0 # configure as p=50% match at 1.0m distance
        k = (- np.log(kp) / kx)
        prob = np.exp(-k*cost)
        return prob

    def get_data(self, resample=True):
        n = len(self.pf_.particles)
        if resample:
            prob = self.pprob()
            self.pf_.resample(prob, noise=(0.05,0.05,0.05))
        prob = self.pprob() # technically inefficient but who cares?

        p = U.renorm(prob, 0.1, 1.0)
        s = 50 * p
        c = np.full((n, 3), [1.0,0.0,0.0])

        p2 = U.renorm(prob, 0.01, 0.2)
        c = np.concatenate([c,p2[:,np.newaxis]], axis=-1)

        x, y = self.pf_.particles[:,:2].T
        return x,y,s,c

    def setup_plot(self):
        """Initial drawing of the scatter plot."""
        x, y, s, c = self.get_data()
        self.scat = self.ax.scatter(x, y, c=c, s=s, animated=True)

        xm, xs = x.mean(), 0.5*x.std()
        ym, ys = y.mean(), 0.5*y.std()
        self.axlim_ = [xm-xs, xm+xs, ym-ys, ym+ys]

        self.ax.axis(self.axlim_)
        self.ax.set_title('Particle Evolution Animation')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.grid(True)

        # For FuncAnimation's sake, we need to return the artist we'll be using
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,

    def update(self, i):
        """Update the scatter plot."""
        x,y,s,c= self.get_data(resample=True)

        # Set x and y data...
        self.scat.set_offsets(np.stack([x,y], axis=-1))
        # Set sizes...
        self.scat._sizes = s
        # Set colors..
        self.scat.set_facecolors(c)
        self.ax.axis(self.axlim_)
        self.ax.set_title('Particle Evolution Iter={}'.format(i))
        self.ax.grid(True)

        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.scat,

    def show(self):
        plt.show()

    def save(self):
        self.ani.save('/tmp/output.gif', writer=animation.ImageMagickWriter(fps=10))

def main():
    n = 1000
    pf = ParticleFilter()
    pf.initialize(size=n, spread=[10.0,0.5])
    anim = PFAnim(pf)
    #anim.show()
    anim.save()

if __name__ == "__main__":
    main()
