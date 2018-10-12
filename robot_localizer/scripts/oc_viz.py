#!/usr/bin/env python
import rospy
from robot_localizer.occupancy_field import OccupancyField
from occupancy_field import OccupancyField as OF2
import numpy as np
from matplotlib import pyplot as plt
import time
from mpl_toolkits.axes_grid1 import make_axes_locatable

def main():
    rospy.init_node('oc_viz')

    of = OccupancyField()
    #print '??'
    #v = of.closest_occ.values()
    #shape = (of.map.info.width, of.map.info.height)
    #v = np.reshape(v, shape)
    #fig, (ax0,ax1) = plt.subplots(1,2)
    #ax0.imshow(v)
    #ax1.imshow(np.reshape(of.map.data, shape).astype(np.float32))

    of2 = OF2()
    p = of.map.info.origin.position
    res = of.map.info.resolution
    print 'o', of.map.info.origin.orientation
    print of.map_.shape
    print('npx', of.map_.size)

    ox, oy = -p.x/res, -p.y/res

    d = of.get_closest_obstacle_distance(3.7, -0.01)
    print 'd', d
    px, py = (3.7 - p.x)/res, (-0.01 - p.y)/res
    print px, py

    m = of.map_
    m[m==-1] = 64 
    m = m.astype(np.uint8)
    v = of.dist_

    fig, (ax0,ax1) = plt.subplots(1,2, sharey=True)
    fig.suptitle('Occupancy Field Visualization')

    ax0.imshow(m, origin='lower', cmap='Greys')
    ax0.plot([ox],[oy], 'r+')
    #ax0.scatter([px],[py])

    p = ax1.imshow(v, origin='lower')
    ax1.plot([ox],[oy], 'r+')
    #ax1.scatter([px],[py])
    ax0.set_xlabel('Original')
    ax1.set_xlabel('Distance Transform')

    divider = make_axes_locatable(ax1)
    cax = divider.append_axes('right', size='5%', pad=0.05)
    fig.colorbar(p, cax=cax)

    plt.show()

if __name__ == "__main__":
    main()
