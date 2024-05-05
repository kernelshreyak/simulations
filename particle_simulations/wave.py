# Wave motion simulation in Visual Python
# Author: Shreyak Chakraborty

#License: GNU GPL v2.0

## The user can change the wavefunction in desired manner and also introduce damping 

print("Wave motion simulation in Visual Python written by Shreyak Chakraborty (C) 2014");

from visual import *

A=10.0;
lmbda=20;
k=2*pi/lmbda;
v=200;
t=0;
dt=0.01;

wave=curve(x=arange(-100,100,0.1),color=color.yellow); # defines the waveform


while 1:
    rate(40);
    wave.y=A*exp(0)*sin(k*(wave.x-v*t)); #The wavefunction : y=(x,t)
    t=t+dt;
