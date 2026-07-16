# Multi Agent simulation for non-interacting crowds
#Author: Shreyak Chakraborty (C) 2015


## License- GNU GPL v2.0


from visual import *
from random import *

seed(50)

scene.autocenter=True
scene.width=1000
scene.height=800

M=[]
N=100

t=0.0
tmax=100

dt=0.01

ground=box(pos=(0,0,0),length=1000,height=0.2,width=1000,color=color.yellow)

for i in range(N):
    M=M+[cylinder(radius=0.5,color=color.red,axis=(0,10,0))]
    M[i].pos=vector(uniform(-100,100),0,uniform(-100,100))
    M[i].vel=vector((0.5)**3*i,0,0.5*i)
    
while t<tmax:
    rate(100)
    for i in range(N):
        M[i].pos=M[i].pos+M[i].vel*dt;
        if M[i].pos.x>500:
            M[i].vel=-M[i].vel
        if M[i].pos.z>500:
            M[i].vel=-M[i].vel
        if M[i].pos.x<-500:
            M[i].vel=-M[i].vel
        if M[i].pos.z<-500:
            M[i].vel=-M[i].vel
        if M[i].pos.mag-M[i-1].pos.mag<10:
            M[i].color=color.blue
            M[i-1].color=color.blue
        
    t=t+dt 
    
    
