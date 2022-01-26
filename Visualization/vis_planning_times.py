import pylab as pl
import os

times=[]
times1=[]
for imageNm in range(1,4):
    imageNum = str(imageNm)

    times.append([])
    times1.append([])

    with open('../output/map' + imageNum + '/Dynamics/times.txt') as f:
        for line in f:
            a = [float(i) for i in line.split()]
            times[imageNm-1].append(a[0]/1000.0)
            times1[imageNm-1].append(a[1]/1000.0)
    f.close()

fig, ax = pl.subplots()
pl.xlabel("Map", fontsize=20);
pl.ylabel("Time (ms)", fontsize=20)
pl.title("Planning time for each map", fontsize=20)
pl.boxplot(times1)
pl.plot([i/10 for i in range(5, 35)], [100 for i in range(5, 35)])
F = pl.gcf()
Size = F.get_size_inches()
F.set_size_inches(Size[0] * 3, Size[1] * 3, forward=True)
pl.tight_layout()
pl.savefig('../output/'+'Planning times'+".png")
pl.show()
# input("hit[enter] to end.")
pl.close('all')
