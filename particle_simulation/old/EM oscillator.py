# Simulation of an Electromagnetic Oscilator in 3-dimensional space
# Author: Shreyak Chakraborty
# Lab-Centre for Theoretical High Energy Physics(CTHEP), Ghaziabad

## License- GNU GPL v2.0


from visual import*
from math import*
from visual.graph import*

# make the scene/world in 3D

scene.autocenter = True
scene.width=640  # horizontal pixels
scene.height=500 # vertical pixels

EC=1.6e-19 #elementary charge (EC)

particle=sphere(pos=(0,0,0), radius=0.16, color=color.cyan,make_trail=True,interval=10)
particle.amp=4
Ewave=sphere(pos=particle.pos,radius=0.05,color=color.green,make_trail=True,interval=10)
Bwave=sphere(pos=particle.pos,radius=0.05,color=color.red,make_trail=True,interval=10)


D_wave=0.01 # wave scale factor

# K=K0*sin(particle.freq*D_wave*t) : K={E,B}   is the modified vacuum equation for the oscillator 

particle.charge=2*EC
particle.freq=400  #frequency in Hertz

w=particle.freq*D_wave
particle.v=vector(0,w,0)

Ewave.charge=Bwave.charge=0

# material K:(k1,k2) sensitive to the EM wave
k1=1.0   # suppression coeff. for E-cmponent 
k2=1.0   # suppression coeff. for B-component
D=10


E0=particle.amp*k1  
B0=particle.amp*k2 


t=0
dt=0.1
tmax=200

while t<tmax:
    rate(100)
    
    #sinusodial variations
    E=E0*sin(w*t)    # oscillating electric field
    B=B0*sin(w*t)    # oscillating magnetic field
    
    particle.pos=particle.pos+particle.v*dt

    if particle.pos.y>particle.amp:
        particle.v=-particle.v
    if particle.pos.y<(-particle.amp):
        particle.v=-particle.v

    Ewave.pos=(t,E,0)
    Bwave.pos=(t,0,B)

    t=t+dt


            
             

             


