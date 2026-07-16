# Simulation of electron gas and interaction with nucleus/ion

#Author: Shreyak Chakraborty (C) 2016

# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad

## License- GNU GPL v2.0



from visual import *
from random import *

scene.autocenter=True
scene.title='Interaction between nuclei/ions and electrons'
scene.width=600
scene.height=600

seed(40)

P=[]    #list for electrons

N=10    #number of elecrons to be simulated

e_SEP=0.8   # Electron Separation Constant(ESC)   

A_SEP=10    #Atomic Separation Constant(ASC)

K=3.4       # force magnitude coeff. (FMC)


L=200       # Length of box

#nucleus/ion definition
nucleus=sphere(pos=(150,-A_SEP,0),radius=1.2,color=color.red)
nucleus.mass=10
nucleus.charge=1000
nucleus.vel=vector(-20,0,0)

# electrons definition
for i in range(N):
    P=P+[sphere(radius=0.06,color=color.yellow,make_trail=True,interval=10)]
    P[i].pos=vector(uniform(-100,80),uniform(-100,80),uniform(-100,80))   # electron distribution
    P[i].mass=0.1
    P[i].charge=-1
    P[i].velocity=P[i].acceleration=vector(0,0,0)
    P[i].force=vector(0,0,0)
    P[i].initial_vel=vector(0,0.0,0)
    P[i].momentum=P[i].initial_vel*P[i].mass



t=0
dt=0.01
tmax=50


while t<tmax:
    rate(100)
    for i in range(N):
        R=nucleus.pos-P[i].pos
        P[i].force=-K*(P[i].charge*nucleus.charge*R/(R.mag**3)) # central force
        P[i].momentum=P[i].momentum+P[i].force*dt
        P[i].pos=P[i].pos+P[i].momentum/P[i].mass*dt

        if nucleus.pos.x>L:
            nucleus.vel=-nucleus.vel
        if nucleus.pos.x<-L:
            nucleus.vel=-nucleus.vel
        nucleus.pos=nucleus.pos+nucleus.vel*dt
    t=t+dt    
                    
                
