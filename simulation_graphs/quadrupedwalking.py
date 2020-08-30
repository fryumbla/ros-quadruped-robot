import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

######################### you can change these values ########################
link1 = 0.4  # link length setting
link2 = 0.4
# th1 = 45.0
# th2 = 45.0
##############################################################################

# value initialize

fr = []
fl = []
br = []
bl = []

stand=0.50

# control loop
step = 100
walk_width = 0.15
rfjoint1=[0.11,stand]
rbjoint1=[-0.11,stand]

# comienzo de la caminata primera position
def beginhold(trajectory, walkwidth,front,i):

    theta = math.pi + i*math.pi/(step-1)
    x = 0.15
    y = stand

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2)-(link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    

    
    if front==True:
        joint1 = rfjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = rbjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    
    print("joint1: ", th1, " Joint2: ", th2)
    return joint1

# comienzo desde atras
def beginstepforward(trajectory, walkwidth,front,i):

    theta = math.pi + i*math.pi/(step-1)
    x = walkwidth/2.0- (walkwidth/2.0 * np.cos(theta))
    y = stand + walkwidth/2.0 * np.sin(theta)

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    

    if front==True:
        joint1 = fjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = bjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    print("joint1: ", th1, " Joint2: ", th2)


# paso atras
def pullback(trajectory, walkwidth,front,i):

    x = 0.15-walkwidth+(i*(walkwidth/(step-1)))
    y = stand
    xi = 0.15-walkwidth
    yi = stand

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    

    th2i = np.arccos(((xi**2)+(yi**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th1i = np.arctan2(yi, xi)-np.arccos(((xi**2)+(yi**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(xi**2+yi**2)))
    if front==True:
        joint1 = rfjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = rbjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    print("joint1: ", th1, " Joint2: ", th2)
    return joint1

# paso atras
def pullbackfoward(trajectory, walkwidth,front,i):

    x = -walkwidth+(i*(walkwidth/(step-1)))
    y = stand
    xi = -walkwidth
    yi = stand

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    

    th2i = np.arccos(((xi**2)+(yi**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th1i = np.arctan2(yi, xi)-np.arccos(((xi**2)+(yi**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(xi**2+yi**2)))
    if front==True:
        joint1 = rfjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = rbjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    print("joint1: ", th1, " Joint2: ", th2)
    return joint1

# paso atras
def pullbackbackward(trajectory, walkwidth,front,i):

    x = 0.15-walkwidth+(i*(walkwidth/(step-1)))
    y = stand
    xi = -walkwidth
    yi = stand

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    

    th2i = np.arccos(((xi**2)+(yi**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th1i = np.arctan2(yi, xi)-np.arccos(((xi**2)+(yi**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(xi**2+yi**2)))
    if front==True:
        joint1 = rfjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = rbjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    
    print("joint1: ", th1, " Joint2: ", th2)
    return joint1

def stepforward(trajectory, walkwidth,front,i):

    theta = math.pi + (i*math.pi/(step-1))
    x = - walkwidth * np.cos(theta)
    y = stand + walkwidth * np.sin(theta)

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    

    if front==True:
        joint1 = fjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = bjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    print("joint1: ", th1, " Joint2: ", th2)


    
print("\nMovement 1")
for i in range(0, step):
    fjoint1 = beginhold(fl,walk_width,True,i)
    beginstepforward(fr,walk_width,True,i)
    bjoint1 = beginhold(br,walk_width,False,i)
    beginstepforward(bl,walk_width,False,i)
rfjoint1=fjoint1
rbjoint1=bjoint1
print("\nMovement 2")
for i in range(0, step):
    fjoint1 = pullback(fr, walk_width,True,i)
    beginstepforward(fl,walk_width,True,i)   
    bjoint1 = pullback(bl, walk_width,False,i)
    beginstepforward(br,walk_width,False,i)   
rfjoint1=fjoint1
rbjoint1=bjoint1
print("\nMovement 3")
for i in range(0, step):
    fjoint1 = pullback(fl, walk_width,True,i)
    stepforward(fr, walk_width,True,i)
    bjoint1 = pullback(br, walk_width,False,i)
    stepforward(bl, walk_width,False,i)
# rfjoint1=fjoint1
# rbjoint1=bjoint1
# print("\nMovement 4")
# for i in range(0, step):
#     fjoint1 = pullbackfoward(fr, walk_width,True,i)
#     stepforward(fl, walk_width,True,i)
#     bjoint1 = pullbackfoward(bl, walk_width,False,i)
#     stepforward(br, walk_width,False,i)
# rfjoint1=fjoint1
# rbjoint1=bjoint1
# print("\nMovement 5")
# for i in range(0, step):
#     fjoint1 = pullbackbackward(fl, walk_width,True,i)
#     stepforward(fr, walk_width,True,i)  
#     bjoint1 = pullbackbackward(br, walk_width,False,i)
#     stepforward(bl, walk_width,False,i)  
# rfjoint1=fjoint1
# rbjoint1=bjoint1

# # create a time array from 0..100 sampled at 0.05 second steps
dt = 0.05

fig = plt.figure(figsize=(16, 8))
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1, 3), ylim=(-0.5, 1.5))
ax.grid()

line1, = ax.plot([], [], 'o-', color='red'      , lw=2)
line2, = ax.plot([], [], 'o-', color='green'    , lw=2)
line3, = ax.plot([], [], 'o-', color='cyan'     , lw=2)
line4, = ax.plot([], [], 'o-', color='yellow'   , lw=2)
line5, = ax.plot([], [], 'o-', color='black'    , lw=2) #line body

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])

    return line1,line2,line3,line4,line5


def animate(i):
    line1x = [fr[i][0][0], fr[i][1][0], fr[i][2][0]]
    line1y = [fr[i][0][1], fr[i][1][1], fr[i][2][1]]
    line2x = [fl[i][0][0], fl[i][1][0], fl[i][2][0]]
    line2y = [fl[i][0][1], fl[i][1][1], fl[i][2][1]]
    line3x = [bl[i][0][0], bl[i][1][0], bl[i][2][0]]
    line3y = [bl[i][0][1], bl[i][1][1], bl[i][2][1]]
    line4x = [br[i][0][0], br[i][1][0], br[i][2][0]]
    line4y = [br[i][0][1], br[i][1][1], br[i][2][1]]
    line5x = [fr[i][0][0], br[i][0][0]]
    line5y = [fr[i][0][1], br[i][0][1]]

    # line6x = []
    # line6y = []
    # for j in range(i):
    #     line6x.append(x5[j])
    #     line6y.append(y5[j])

    line1.set_data(line1x, line1y)
    line2.set_data(line2x, line2y)
    line3.set_data(line3x, line3y)
    line4.set_data(line4x, line4y)
    line5.set_data(line5x, line5y)

    return line1,line2,line3,line4,line5


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(fr)),interval=1, blit=True, init_func=init, repeat=True)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
