from visual import *

scene.autocenter = True
scene.width=1024
scene.height=768

p=sphere(pos=(0,0,0), radius=0.2, color=color.yellow,make_trail=True,interval=10)
ret=box(pos=(50,0,0),size=(1,1,1),color=color.green)
p.vel=vector(0,0,0)   # initial velocity=0
p.mass=1
p.charge=1  #1=e

E=vector(10,0,0)
B=vector(0,0,10)
F1=p.charge*E
F2=p.charge*cross(p.vel,B)
t=0
dt=0.02  # discrete time step


while t<500:
    rate(100)
    if p.pos.x==ret.pos:
        Fmag=mag(F2)
        v=(Fmag/p.mass)*dt
        
    else Fmag=F1.x
    
   
        
    
    t=t+dt



