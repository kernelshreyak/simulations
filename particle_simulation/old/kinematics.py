
# Kinematics simulation in VPython
# Author: Shreyak Chakraborty
# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad  

from visual import*

# Body definition(s)
a=box(pos=(0,0,0), size=(2,2,2), color=color.green)
b=box(pos=(0,0,0), size=(2,2,2), color=color.red)

ground=box(pos=(0,0,0), size=(200,0.2,50), color=color.yellow)
        
a.mass=100
b.mass=50

#world parameters
a.vel=b.vel=vector(0,0,0)
F=vector(5,0,0)
t=0

dt=0.05 # discrete time step

Fmag=F.x

while t<100:
    rate(100)
    va=(Fmag/a.mass)*dt
    vb=(Fmag/b.mass)*dt
    a.vel=a.vel+vector(va,0,0)
    b.vel=b.vel+vector(vb,0,0)
    
    a.pos=a.pos + a.vel*dt
    b.pos=b.pos + b.vel*dt
    t=t+dt






