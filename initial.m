massKg = 2500; % consider raptor f150, kinda of my favourite car
gravityMps2 = 9.81; % gravity
mu = 0.7; %% dry, wet = 0.4-0.6 and icy = 0.1to 0.2 and snow = 0.2 and 0.3 
coeffDrag = 0.28; %drag coefficient 
airDensityKgpm3= 1.225; % air density
areaM2 = 2.5;  % area required for
dt_pid = 1e-2; % smapling time for the controller
pole_placement = -0.9; % pole selection for feedback law
v0 = 10; %% initial velocity
a_max = 3.5;
a_min = -3.5;
massKg1 = 2400;