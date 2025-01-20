# Simulation of rotating bodies via rotation of axial vector
# Author: Shreyak Chakraborty
# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad  

from visual import*

body1=box(pos=(0,0,0), size=(2,2,2), color=color.green)
body2=box(pos=(8,0,0), size=(1,1,1), color=color.yellow)
body1.axis=vector(1,0,0)
body2.axis=vector(2,3,5)

t=0
while t<50:
    rate(2000)
    body1.rotate(axis=(1,0,0),angle=pi/4)
    body2.rotate(axis=(2,3,5),angle=pi/10)
