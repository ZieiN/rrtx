import pylab as pl
import os
width = 600
height = 600
for imageNm in range(2,4):
    imageNum = str(imageNm)

    stX = []
    stY = []

    ndX = []
    ndY = []

    with open('../input/map' + imageNum + '/inputs.txt') as f:
        for line in f:
            a = [float(i) for i in line.split()]
            stX.append(a[0])
            stY.append(a[1])
            ndX.append(a[3])
            ndY.append(a[4])
    f.close()

    times = []
    times1 = []
    with open('../output/map' + imageNum + '/Dynamics/times.txt') as f:
        for line in f:
            a = [float(i) for i in line.split()]
            times.append(a[0])
            times1.append(a[1])
    f.close()

    numOfSeconds = 150
    numOfTests = 20

    counter = 0
    for id in range(0, 0):
        shiftsX = []
        shiftsY = []
        shiftsX1 = []
        shiftsY1 = []
        with open('../output/map' + imageNum + '/Dynamics/shifts-'+str(id)+'.txt') as f:
            for line in f:
                a = [float(i) for i in line.split()]
                shiftsX.append(a[0])
                shiftsY.append(a[1])
                shiftsX1.append(a[2])
                shiftsY1.append(a[3])
            f.close()
        counter += len(shiftsX)
    for id in range(10, numOfTests):
        shiftsX = []
        shiftsY = []
        shiftsX1 = []
        shiftsY1 = []
        with open('../output/map' + imageNum + '/Dynamics/shifts-'+str(id)+'.txt') as f:
            for line in f:
                a = [float(i) for i in line.split()]
                shiftsX.append(a[0])
                shiftsY.append(a[1])
                shiftsX1.append(a[2])
                shiftsY1.append(a[3])
            f.close()
        distraction = []
        with open('../output/map' + imageNum + '/Dynamics/distraction-'+str(id)+'.txt') as f:
            for line in f:
                a = [float(i) for i in line.split()]
                distraction.append(a[0])
            f.close
        solPathX = []
        solPathY = []
        print(id, len(shiftsX))
        for id1 in range(0, len(shiftsX)-1):
            pl.clf()
            pl.axis('equal')
            pl.plot([0, 0, 600, 600, 0], [600, 0, 0, 600, 600], "r-")
            b = []
            c = []
            d = []
            e = []
            ff = []
            gg = []
            cnt = 0
            with open('../output/map' + imageNum + '/Dynamics/outTree' + str(id) + '-' + str(id1)+'.txt') as f:
                for line in f:
                    if(cnt%1==0):
                        a = [float(i) for i in line.split()]
                        if len(a) == 4:
                            b.append(a[0])
                            c.append(a[1])
                            d.append(a[2])
                            e.append(a[3])
                        else:
                            ff.append(a[0])
                            gg.append(a[1])
                    cnt += 1
            f.close()

            # pl.plot(b, c, 'b.')
            # pl.plot(d, e, 'b.')
            print(len(b))
            pl.plot([b, d], [c, e], 'y-')
            pl.plot(ff, gg, '.', markerfacecolor='white', markeredgecolor='blue', markersize=12)

            # pl.plot(solPathX, solPathY, 'b-', linewidth=4)
            solPathX = []
            solPathY = []
            solPathX1 = []
            solPathY1 = []
            solPathX2 = []
            solPathY2 = []
            cnt = 0
            with open('../output/map' + imageNum + '/Dynamics/outSol' + str(id) + '-'+str(id1+1)+'.txt') as f:
                for line in f:
                    if(cnt%1==0):
                        a = [float(i) for i in line.split()]
                        # pl.plot([a[0], a[2]], [a[1], a[3]], 'g-', linewidth=4)
                    cnt += 1
                    if len(a)==0:
                        continue
                    solPathX.append(a[0])
                    solPathY.append(a[1])
                    if len(a)>2:
                        solPathX1.append(a[2])
                        solPathY1.append(a[3])
                        solPathX2.append(a[4])
                        solPathY2.append(a[5])
            f.close()

            pl.plot(solPathX, solPathY, 'g-')#, linewidth=4)
            pl.plot(solPathX1, solPathY1, 'b-')#, linewidth=4)
            pl.plot(solPathX2, solPathY2, 'r-')#, linewidth=4)

            imgx = []
            imgy = []
            with open("../output/map" + imageNum + "/Dynamics/outMap" + str(id) + "-"+ str(id1)+".txt") as f:
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
            # dx = [0.5, 0.5, -0.5, -0.5]
            # dy = [0.5, -0.5, -0.5, 0.5]
            # for x in range(len(imgx)):
            #     gx = []
            #     gy = []
            #     for d in range(4):
            #         gx.append(imgx[x]+dx[d])
            #         gy.append(imgy[x]+dy[d])
            #     pl.plot(gx, gy, 'r-')

            pl.plot(stX[id]+shiftsX1[id1], stY[id]+shiftsY1[id1], 'm.', markersize=16)
            pl.plot(ndX[id]-shiftsX[id1], ndY[id]-shiftsY[id1], 'r.', markersize=16)
            pl.title("Test-" + str(id)+"-at time:"+str(id1)+",Planning Time:"+str(times1[counter]/1000.0)+
                     "ms\nRewiring Time:"+str(times[counter]/1000.0)+"ms,...Traj difference="+str(distraction[id1+1]), fontsize=20)
            counter += 1
            print("map:",imageNum,", test:",id,", time:",id1, "saving..")
            F = pl.gcf()
            Size = F.get_size_inches()
            F.set_size_inches(Size[0] * 3, Size[1] * 3, forward=True)
            pl.tight_layout()
            if not os.path.exists('../output/ImagesMap' + imageNum + "/test" + str(id) + "/"):
                os.mkdir('../output/ImagesMap' + imageNum + "/test" + str(id) + "/")
            pl.savefig('../output/ImagesMap' + imageNum + "/test" + str(id) + "/"+str(id1)+".png")
            # pl.show()
            # input("hit[enter] to end.")
            pl.close('all')
        counter += 1
