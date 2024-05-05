
# Simulation of interactions of electrons with atomic nuclei
#Author: Shreyak Chakraborty (C) 2016

# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad

## License- GNU GPL v2.0

# Note: Multiple nuclei can be added and electron beam profile can be customised


from visual import *
from random import *

scene.autocenter=False
scene.title='Interaction between nuclei and electrons'
scene.width=1024
scene.height=768

seed(40)

P=[]    #list for electrons

N=30    #number of elecrons to be simulated

e_SEP=0.8   # Electron Separation Constant(ESC)   

A_SEP=10    #Atomic Separation Constant(ASC)

K=3.4       # force magnitude coeff. (FMC)


#nucleus definition
nucleus1=sphere(pos=(50,-A_SEP,0),radius=1.2,color=color.red)
nucleus1.mass=200
nucleus1.charge=500

nucleus1=sphere(pos=(50,-A_SEP,0),radius=1.2,color=color.red)
nucleus1.mass=200
nucleus1.charge=100


# electrons definition
for i in range(N):
    P=P+[sphere(radius=0.06,color=color.yellow,make_trail=True,interval=10)]
    P[i].pos=vector(uniform(-100,80),uniform(-100,80),uniform(-100,80))   # electron distribution
    P[i].mass=0.2
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
        R1=nucleus1.pos-P[i].pos
        R2=nucleus2.pos-P[i].pos
        P[i].force=-K*(P[i].charge*nucleus1.charge*R1/(R1.mag**3)+P[i].charge*nucleus2.charge*R2/(R2.mag**3)) # central force
        P[i].momentum=P[i].momentum+P[i].force*dt
        P[i].pos=P[i].pos+P[i].momentum/P[i].mass*dt
        
    t=t+dt    




                       
                       
    
                       
                       
                      
                      
                      
                      
                      
