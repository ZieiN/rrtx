import pylab as pl
width = 600
height = 600
imageNum = "3"

stX = []
stY = []

ndX = []
ndY = []

with open('../input/map' + imageNum + '/inputs.txt') as f:
    for line in f:
        a = [float(i) for i in line.split()]
        stX.append(a[1])
        stY.append(599-a[0])
        ndX.append(a[4])
        ndY.append(599-a[3])
f.close()

times = []
with open('../output/map' + imageNum + '/times.txt') as f:
    for line in f:
        a = [float(i) for i in line.split()]
        times.append(a[0])
f.close()

for id in range(len(stX)):
    pl.clf()
    pl.axis('equal')
    pl.plot([0, 0, 600, 600, 0], [600, 0, 0, 600, 600], "r-")
    b = []
    c = []
    d = []
    e = []
    ff = []
    gg = []
    with open('../output/map' + imageNum + '/outTree' + str(id) + '.txt') as f:
        for line in f:
            a = [float(i) for i in line.split()]
            if len(a) == 4:
                b.append(a[0])
                c.append(a[1])
                d.append(a[2])
                e.append(a[3])
            else:
                ff.append(a[0])
                gg.append(a[1])
    f.close()

    pl.plot(b, c, 'b.')
    pl.plot(d, e, 'b.')
    pl.plot([b, d], [c, e], 'y-')
    pl.plot(ff, gg, '.', markerfacecolor='white', markeredgecolor='blue', markersize=12);
    with open('../output/map' + imageNum + '/outSol' + str(id) + '.txt') as f:
        for line in f:
            a = [float(i) for i in line.split()]
            pl.plot([a[0], a[2]], [a[1], a[3]], 'g-', linewidth=4)
    f.close()

    imgx = []
    imgy = []
    with open('../input/map' + imageNum + '/image.txt') as f:
        cnt = 0
        for line in f:
            if cnt == 0:
                cnt += 1
                continue
            arr = [int(i) for i in line.split()]
            for i in range(len(arr)):
                if arr[i] == 1:
                    imgx.append(i)
                    imgy.append(height - cnt)
            cnt += 1
    f.close()
    pl.plot(imgx, imgy, 'k.')

    pl.plot(stX[id], stY[id], 'm.', markersize=16)
    pl.plot(ndX[id], ndY[id], 'r.', markersize=16)
    pl.title("Test-" + str(id)+", Planning Time:"+str(times[id]/1000.0)+"ms", fontsize=20)
    print("saving..")
    F = pl.gcf()
    Size = F.get_size_inches()
    F.set_size_inches(Size[0] * 3, Size[1] * 3, forward=True)
    pl.tight_layout()
    pl.savefig('../output/ImagesMap' + imageNum + "/test" + str(id) + ".png")
    # pl.show()
    # input("hit[enter] to end.")
    pl.close('all')
