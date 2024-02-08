import pylab as pl
import os
width = 600
height = 600

with open('tmp.txt') as f:
    for line in f:
        a = [float(i) for i in line.split()]
        x = [a[i] for i in range(0, len(a), 2)]
        y = [a[i] for i in range(1, len(a), 2)]
        pl.plot(x, y, '.')
f.close()

pl.axis('equal')
pl.show()