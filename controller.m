 function [newErrorRPY, u] = controller(prevErrorRPY,state, params,gains)
%CONTROLLER  Controller for the VTOL; implements PI control on RPY
%
%  state: The current state of the robot with the following fields:
%  state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%  state.rot = [phi; theta; psi], state.omega = [p; q; r]

% INPUT
%
% OUTPUT
% params: robot parameters
%

% PI gains  
Kpphi = gains(1);
Kiphi = gains(2);
Kptheta = gains(3);
Kitheta = gains(4);
Kppsi = gains(5);
Kipsi = gains(6);

  

% Accumelating errors in RPY for integral term
  ephi = prevErrorRPY(1);
  ephi = ephi + state.rot(1);
  newErrorRPY(1,1) = ephi;

  etheta = prevErrorRPY(2);
  etheta = etheta + state.rot(2);
  newErrorRPY(2,1) = etheta;
  
  epsi = prevErrorRPY(3);
  epsi = epsi + state.rot(3);
  newErrorRPY(3,1) = epsi;

% Desired RPY are zero
  u =[(Kpphi*(-state.rot(1))     +     Kiphi*(-ephi))*params.I(1,1)/params.l;
	  (Kptheta*(-state.rot(2)) + Kitheta*(-etheta))*params.I(2,2)/params.L;
	  (Kppsi*(-state.rot(3))     +    Kipsi*(-epsi))*params.I(3,3)/params.L];




end
