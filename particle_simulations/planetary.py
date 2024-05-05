# Simulation of orbital motion under gravitational field
# Author: Shreyak Chakraborty
# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad

## License- GNU GPL v2.0

from visual import*

scene.autocenter = True
scene.width=640
scene.height=500

particle=sphere(pos=(2,3,4),radius=0.3,color=color.green,make_trail=True,interval=10)
particle.mass=10
particle.velocity=particle.acceleration=vector(0,0,0)
particle.force=vector(0,0,0)

source=sphere(pos=(10,0,0),radius=1.2,color=color.yellow)
source.mass=2000

particle.initial_vel=vector(2,2,0)
particle.p=particle.initial_vel*particle.mass

K=0.034

t=0
dt=0.01
tmax=200

while t<tmax:
    rate(100)
    R=source.pos-particle.pos
    particle.force=K*particle.mass*source.mass*R/(R.mag**3)
    particle.p=particle.p+particle.force*dt
    particle.pos=particle.pos+particle.p/particle.mass*dt
    t=t+dt
    




