
# Simulation of proton-neutron interaction via strong nuclear force using
# Yukawa Well type nuclear potential : V(r)=-(V_0)*exp(-r/a)/r

#Author: Shreyak Chakraborty (C) 2015

# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad

## License- GNU GPL v2.0


from visual import *
from math import *


scene.autocenter=True
scene.width=1024
scene.height=768

# proton definition
proton=sphere(pos=(9,0,0),radius=0.09,color=color.magenta,make_trail=True,interval=10)
proton.mass=80
proton.charge=1
proton.velocity=proton.acceleration=vector(0,0,0)
proton.force=vector(0,0,0)
proton.initial_vel=vector(0,0.0,0.0)
proton.momentum=proton.initial_vel*proton.mass


# neutron definition
neutron=sphere(pos=(8,0,0),radius=0.1,color=color.blue,make_trail=True,interval=10)
neutron.mass=100
neutron.charge=0
neutron.velocity=neutron.acceleration=vector(0,0,0)
neutron.force=vector(0,0,0)
neutron.initial_vel=vector(0.02,0.1,0.03)
neutron.momentum=neutron.initial_vel*neutron.mass




Vo=10
a=5 #in femtometer

t=0
dt=0.01
tmax=100

while t<tmax:
    rate(100)    
    r=proton.pos-neutron.pos
    proton.force=-Vo*math.exp(-r.mag/a)*r*(1/(a*r.mag**2)+1/(r.mag**3))
    neutron.force=Vo*r*math.exp(-r.mag/a)*(1/(a*r.mag**2)+1/(r.mag**3))
    proton.momentum=proton.momentum+proton.force*dt
    proton.pos=proton.pos+proton.momentum/proton.mass*dt
    neutron.momentum=neutron.momentum+neutron.force*dt
    neutron.pos=neutron.pos+neutron.momentum/neutron.mass*dt

    t=t+dt   
    
    
    
        
