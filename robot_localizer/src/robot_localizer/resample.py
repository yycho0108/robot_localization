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
            ps2[i] = ps[j]
            ws2[i] = ws[j] * i_scale

            u0 = u0 + (scale - u0) * nform(n-i-1)

        return ws2, ps2

def main():
    from matplotlib import pyplot as plt
    n = 1000
    ws = np.random.uniform(size=n)
    ws = np.sort(ws, -1)
    ps = np.arange(n)
    ws2, ps2 = resample(ps,ws)

    ws_r = np.convolve(ws, [0.25,0.5,0.25], 'same')
    ws_r = ws_r/ws_r.sum()
    plt.hist(ps2, color='g', density=True)
    plt.scatter(ps, ws_r, color='r')
    plt.show()

if __name__ == '__main__':
    main()
