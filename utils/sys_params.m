function params = sys_params()


m = 0.18; % kg
g = 9.81; % m/s/s

% modify I according to vtol solidworks model 
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.l = 0.086; % modify
params.L =1;% distance between rotors

params.minT = 0.0;
params.maxT = m*g/2;%???????
params.maxalpha = 8*pi/180;

params.R = .1;%m
params.A = 150e-4;%m2
end
