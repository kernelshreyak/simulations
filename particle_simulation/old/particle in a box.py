
# Simulation of particle motion in a box
# Author: Shreyak Chakraborty
# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad

## License- GNU GPL v2.0


from visual import*

p1=sphere(pos=(-5,0,0), radius=0.2, color=color.yellow)
p2=sphere(pos=(5,5,0), radius=0.2, color=color.red)

wallR=box(pos=(6,0,0), size=(0.2,12,12), color=color.cyan)
wallL=box(pos=(-6,0,0), size=(0.2,12,12), color=color.cyan)

p1.velocity=vector(50,0,0)
p2.velocity=vector(-50,0,0)




dt=0.005
t=0

while t<100:
    rate(100)
    if p1.pos.x>wallR.pos.x:
        p1.velocity=-p1.velocity
        p2.velocity=-p2.velocity
    if p1.pos.x<wallL.pos.x:
        p1.velocity=-p1.velocity
        p2.velocity=-p2.velocity

    p1.pos=p1.pos + p1.velocity*dt
    p2.pos=p2.pos + p2.velocity*dt
    t=t+dt
    
