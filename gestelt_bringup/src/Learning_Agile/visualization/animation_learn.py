import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation

fig = plt.figure()
ax = plt.axes()
scats = []
a = np.random.rand(8,18)
im = ax.imshow(a, cmap="YlGn", vmin=0, vmax=3, extent=[0,2000,0,1000])
plt.xticks([])
plt.yticks([])

def randpair(n):
    x,y=[],[]
    for i in range(n):
        x.append(np.random.randint(100,1900))
        y.append(np.random.randint(100,900))
    return x,y

def animate(i):
    global scats
    # first remove all old scatters
    for scat in scats:
        scat.remove()
    scats=[]
    # now draw new scatters
    points = np.random.randint(5,size=10)
    for j in points: 
        x, y = randpair(j)
        if len(x) > 0 :
            scats.append(ax.scatter(x,y,color='r',s=18))  

anim = matplotlib.animation.FuncAnimation(fig, animate, 50,
                                interval=1000, blit=False)

writer = matplotlib.animation.FFMpegWriter(fps=15, 
            codec="h264", 
            extra_args=["-preset", "veryslow","-crf","0"])
anim.save(__file__+".mp4", writer=writer)

plt.show()