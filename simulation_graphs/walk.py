import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

######################### you can change these values ########################
link1 = 0.4  # link length setting
link2 = 0.4
th1 = 45.0
th2 = 45.0
##############################################################################

# value initialize
fr1 = [0.0, 0.5]
fr2 = [0.0, 0.0]
fr3 = [0.0, 0.0]

fl1 = [0.0, 0.5]
fl2 = [0.0, 0.0]
fl3 = [0.0, 0.0]

fr=[]
fr1_t = []
fr2_t = []
fr3_t = []

fl=[]
fl1_t = []
fl2_t = []
fl3_t = []

# control loop
step = 200

def pullback(trajectory,walkwidth):
    for i in range(0, step):
        x = -walkwidth+(i*(walkwidth*2/(step-1)))
        y = 0.65

        th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
        print th2
        th2_i = -th2
        th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

        print th1
        # print 'th1:',th1*180/math.pi,' th2:',(th2*180/math.pi)-45

        joint1 = [0.0, 0.0]
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1,joint2,joint3])

def stepforward(trajectory, walkwidth):
    for i in range(0, step):
        theta = math.pi + i*math.pi/(step-1)
        x = - walkwidth * np.cos(theta)
        y = 0.65 + walkwidth * np.sin(theta)

        th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
        th2_i = -th2
        th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))
        # print 'th1:',th1*180/math.pi,' th2:',(th2*180/math.pi)-45

        joint1 = [0.0, 0.0]
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1,joint2,joint3])

stepforward(fr,0.15)
pullback(fr,0.15)

pullback(fl,0.15)
stepforward(fl,0.15)

# # create a time array from 0..100 sampled at 0.05 second steps
dt = 0.05

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
ax.grid()

line1, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], 'o-', lw=2)


def init():
    line1.set_data([], [])
    line2.set_data([], [])

    return line1, line2


def animate(i):
    line1x = [fr[i][0][0], fr[i][1][0], fr[i][2][0]]
    line1y = [fr[i][0][1], fr[i][1][1], fr[i][2][1]]
    line2x = [fl[i][0][0], fl[i][1][0], fl[i][2][0]]
    line2y = [fl[i][0][1], fl[i][1][1], fl[i][2][1]]

    # line6x = []
    # line6y = []
    # for j in range(i):
    #     line6x.append(x5[j])
    #     line6y.append(y5[j])

    line1.set_data(line1x, line1y)
    line2.set_data(line2x, line2y)

    return line1, line2


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(fr)),
                              interval=10, blit=True, init_func=init, repeat=True)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
