# Simulation of undamped Spring compression and extension under external force
# {Special quick nested loop algorithm devised}
# Author: Shreyak Chakraborty

# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad

## License- GNU GPL v2.0

from visual import*

#spring initialisation
L0=vector(0,80,0) # initial length
spring=helix(pos=(0,-20,0),axis=L0,radius=1.5,coils=10,thickness=0.8,color=color.orange)
spring.const=14
spring.extension=vector(0,0,0)

force=vector(0,20,0)          # applied force on spring

extmin=vector(0,5,0)
extmax=L0

t=0
dt=0.1
tmax=50

while t<tmax:
    rate(100)
    L=force.mag/spring.const          #compression using Hooke's Law
    spring.extension=vector(0,L,0)
    spring.axis-=spring.extension
    if spring.axis.y<extmin.y:
        t=0
        while t<tmax:
            rate(100)
            L=force.mag/spring.const  #extension using Hooke's Law
            spring.extension=vector(0,L,0)
            spring.axis+=spring.extension
            if spring.axis.y>extmax.y:
                break
            t+=dt
        
    t+=dt



