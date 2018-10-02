import numpy as np
from matplotlib import pyplot as plt

def nform(n):
    return 1.0 - pow(np.random.uniform(), 1.0/(n+1))

class Resampler(object):
    def __init__(self):
        pass
    def __call__(self, ws, ps, n=None):
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

        for i in range(n):
            while (t + ws[j] < u0 and j < m):
                t += ws[j]
                j += 1
            ps2[i] = ps[j]
            ws2[i] = ws[j] * i_scale

            u0 = u0 + (scale - u0) * nform(n-i-1)

        return ws2, ps2

def main():
    sampler = Resampler()
    n = 100
    ws = np.random.uniform(size=n)
    ws = np.sort(ws)
    ps = np.arange(n)
    ws2, ps2 = sampler(ws,ps)

    plt.scatter(ps, ws/ws.sum(), color='r')
    plt.hist(ps2, color='g', density=True)
    plt.show()

if __name__ == '__main__':
    main()
