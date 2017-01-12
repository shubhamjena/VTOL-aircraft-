function simulation_3d(gains)


tspan = 0:.01:20;
disp('Setting initial conditions...');
% initial state 
x0    = init_state();



% ************************* RUN SIMULATION *************************
disp('Simulation Running....');

% xsave : len(tspan)*13 matrix
Xsave = rk4t(@EOM,tspan,x0,gains);


% ************************* POST PROCESSING *************************

% plotting x,y,z vs t
figure;

subplot(3,1,1),plot(tspan,Xsave(:,1));xlabel('t');ylabel('X');
title('really stupid results 1');
subplot(3,1,2),plot(tspan,Xsave(:,2));xlabel('t');ylabel('Y');
subplot(3,1,3),plot(tspan,Xsave(:,3));xlabel('t');ylabel('Z');


figure;

subplot(3,1,1),plot(tspan,Xsave(:,4));xlabel('t');ylabel('\phi');
title('really stupid results 2');
subplot(3,1,2),plot(tspan,Xsave(:,5));xlabel('t');ylabel('\theta');
subplot(3,1,3),plot(tspan,Xsave(:,6));xlabel('t');ylabel('\psi');

disp('simulation done');
end
