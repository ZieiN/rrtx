import pylab as pl
import os

diffs = []
for imageNm in range(1,4):
    imageNum = str(imageNm)

    diffs.append([])
    numOfTests = 20
    for id in range(10, numOfTests):
        with open('../output/map' + imageNum + '/Dynamics/distraction-'+str(id)+'.txt') as f:
            for line in f:
                a = [float(i) for i in line.split()]
                if a[0]>=-0.5:
                    diffs.append(a[0])
            f.close


fig, ax = pl.subplots()
pl.xlabel("Map", fontsize=20);
pl.ylabel("Lock-step Euclidean Dist", fontsize=20)
pl.title("Difference between consecutive trajectories for each map", fontsize=20)
pl.violinplot(diffs)
F = pl.gcf()
Size = F.get_size_inches()
F.set_size_inches(Size[0] * 3, Size[1] * 3, forward=True)
pl.tight_layout()
pl.savefig('../output/'+'Planning times'+".png")
pl.show()
# input("hit[enter] to end.")
pl.close('all')
