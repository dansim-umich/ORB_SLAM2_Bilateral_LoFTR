import numpy as np

def display(left,right):
    print("left size:\n", left)
    print("right size:\n", right)
    x = left + right
    y = x.shape
    x = x.flatten()
    x  = np.insert(x, 0, y[0])
    return x.tolist()