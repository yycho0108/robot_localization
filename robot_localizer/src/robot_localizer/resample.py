import numpy as np

def nform(n):
    return 1.0 - pow(np.random.uniform(), 1.0/(n+1))

def resample(ps, ws, n=None):
    m = len(ps)
    if n is None:
        n = m
    scale = np.sum(ws)
    i_scale = (1.0 / scale)
    u0 = nform(n-1) * scale
    i, j = 0,0
    t = 0

    ps2 = np.empty_like(ps)
    ws2 = np.empty_like(ws)

    for i in range(n): # TODO : potentially slow?
        while (t + ws[j] < u0 and j < m):
            t += ws[j]
            j += 1
        ps2[i,:] = ps[j,:]
        ws2[i] = ws[j] * i_scale

        u0 = u0 + (scale - u0) * nform(n-i-1)

    return ps2, ws2

def main():
    from matplotlib import pyplot as plt

    #n = 10
    for n in np.logspace(1,3,num=20).astype(np.int32):
        ws = np.random.uniform(size=n)
        ws = np.sort(ws, -1)
        ps = np.arange(n).reshape((n,1))
        ps2, ws2 = resample(ps,ws)

        #ws_r = np.convolve(ws, [0.25,0.5,0.25], 'same')
        #ws_r = ws_r/ws_r.sum()
        ws_r = ws / ws.sum()

        #_, bins, _ = plt.hist(ps, color='b', density=True, label='initial')
        #plt.hist(ps2, bins, color=(1,0,0,0.5), density=True, label='resample')
        plt.clf()
        plt.hist([ps[:,0],ps2[:,0]], density=True, label=['initial','resample'])
        plt.plot(ps, ws_r, color=(0,1,0,1.0), label='weight')
        plt.legend(loc=2)
        plt.title('Particle Resampling Results, n={0:04}'.format(n))
        #plt.show()
        plt.savefig('/tmp/{0:04}.png'.format(n))

if __name__ == '__main__':
    main()
