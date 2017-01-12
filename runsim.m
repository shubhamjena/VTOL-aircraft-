close all;
clear;

addpath('utils');

% controller gains
Kpphi = 0;
Kiphi = 0;
Kptheta = 0;
Kitheta = 0;
Kppsi = 0;
Kipsi = 0;

gains(1)= Kpphi;
gains(2)= Kiphi;
gains(3)= Kptheta;
gains(4)= Kitheta;
gains(5)= Kppsi;
gains(6)= Kipsi;

simulation_3d(gains);
