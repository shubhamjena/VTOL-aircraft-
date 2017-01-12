function sdot = EOM( t,s,gains)
% quadEOM_readonly calculate the derivative of the state vector
%
% INPUTS:
% s      - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% u      - 3 x 1, control output from controller 
% gains  - 1x 6, PI gains
% OUTPUTS:
% sdot   - 13 x 1, derivative of state vector s
%


%%testing



% crap
params = sys_params();


%************ EQUATIONS OF MOTION ************************
% foreigner
% convert state to quad stuct for control
current_state = stateToQd(s);


%More crap
persistent err;
if(t==0),err = zeros(3,1);end


% get control outputs
[err_update,u] = controller(err, current_state, params,gains);


% update error
err =err+err_update;
%************************************************************


% Expressing T1,T2,alpha1,alpha2 in terms of control input vector u
 T1 = (params.mass*params.gravity-u(2))/2;
 T2 = (params.mass*params.gravity+u(2))/2;
 alpha1 = (pi*params.R*params.R/params.A)*(u(1)+u(3));
 alpha2 = (pi*params.R*params.R/params.A)*(u(1)-u(3));
 
 T1_clamped = min(max(params.minT,T1), params.maxT);
 T2_clamped = min(max(params.minT, T2), params.maxT);
 alpha1_clamped = sign(alpha1)*min(params.maxalpha,abs(alpha1));
 alpha2_clamped = sign(alpha2)*min(params.maxalpha,abs(alpha2));

 u_clamped(1,1) = (params.mass*params.gravity/2)*(params.A/(pi*params.R*params.R))*(alpha1_clamped+alpha2_clamped); 
 u_clamped(2,1) = (params.mass*params.gravity/2)*(params.A/(pi*params.R*params.R))*(alpha1_clamped+alpha2_clamped); 
 u_clamped(3,1) = T2_clamped - T1_clamped;

 
 
% Assign states
x = s(1);
y = s(2);
z = s(3);
xdot = s(4);
ydot = s(5);
zdot = s(6);
qW = s(7);
qX = s(8);
qY = s(9);
qZ = s(10);
p = s(11);
q = s(12);
r = s(13);

quat = [qW; qX; qY; qZ];
bRw = QuatToRot(quat);
wRb = bRw';



% Force due to keel
F1 = T1_clamped*sin(alpha1_clamped)*params.A/(pi*params.R*params.R);
F2 = T2_clamped*sin(alpha2_clamped)*params.A/(pi*params.R*params.R);
% Acceleration
accel = (1 / params.mass) *( (wRb * ...
         [0; 
          F1*cos(alpha1_clamped)+F2*cos(alpha2_clamped);
          T1_clamped+T2_clamped-F1*sin(alpha1_clamped)-F2*sin(alpha2_clamped)])...
       - [0; 0; params.mass * params.gravity]);

% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * quat + K_quat*quaterror * quat;

% Angular acceleration
omega = [p;q;r];
pqrdot   = params.invI * (u_clamped - cross(omega, params.I*omega));

% Assemble sdot
sdot = zeros(13,1);
sdot(1)  = xdot;
sdot(2)  = ydot;
sdot(3)  = zdot;
sdot(4)  = accel(1);
sdot(5)  = accel(2);
sdot(6)  = accel(3);
sdot(7)  = qdot(1);
sdot(8)  = qdot(2);
sdot(9)  = qdot(3);
sdot(10) = qdot(4);
sdot(11) = pqrdot(1);
sdot(12) = pqrdot(2);
sdot(13) = pqrdot(3);

end
