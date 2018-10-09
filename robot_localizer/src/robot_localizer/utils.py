import numpy as np

def anorm(x):
    return (x+np.pi) % (2*np.pi) - np.pi

def adiff(a,b):
    return anorm(a-b)

def amean(x, w=None):
    if w is not None:
        s = np.sin(x)
        c = np.cos(x)
        return np.arctan2((s*w).sum(), (c*w).sum())
    else:
        return np.arctan2(np.sin(x).sum(),np.cos(x).sum())
