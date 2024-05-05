
# Experimental Simulation of motion of a particle under a central force field F=F(r)
# Author: Shreyak Chakraborty
# Lab-Centre for Theoretical High Energy Physics (CTHEP), Ghaziabad  

## License- GNU GPL v2.0

# NOTE:- The user can experiment with different attractive and repulsive central force fields


from visual import*

scene.autocenter = True
scene.width=1024
scene.height=768

particle=sphere(pos=(-30,8,0),radius=0.1,color=color.green,make_trail=True,interval=10)
particle.mass=10
particle.velocity=particle.acceleration=vector(0,0,0)
particle.force=vector(0,0,0)

source=sphere(pos=(10,0,0),radius=1.2,color=color.yellow)
source.mass=20000

particle.initial_vel=vector(10,0,0)
particle.p=particle.initial_vel*particle.mass

K=0.034 # force magnitude coeff. (FMC)

t=0
dt=0.01
tmax=200

# Momentum update-based loop algorithm

while t<tmax:
    rate(100)
    R=source.pos-particle.pos
    particle.force=K*particle.mass*source.mass*R  # the central force law
    particle.p=particle.p+particle.force*dt
    particle.pos=particle.pos+particle.p/particle.mass*dt
    t=t+dt
    




