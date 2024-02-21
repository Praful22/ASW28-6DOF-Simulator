%% Praful Sigdel
% Initialization of parameters for simulink models.
clear all;
clc

% NON-zero moments and forces case

% Initializing Vehicle orientation
roll = 0.5; 
pitch = 0.0;
yaw = 0.0; 

% External forces
Fx = 600;
Fy = 10000;
Fz = -9.81;

% External Moments
LL = 1000;
MM = 10;
NN = 1000;

% Initial Quaternion 
q0 = angle2quat(yaw,pitch,roll);

% initial Position/Condition for Position Integrator.
x0 = [6378100;0;0]; 

%Define Initial DCM for Poisson's Kinematical Equations
C1 = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0;0 0 1];
C2 = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
C3 = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];

% Convert DCM
Cnb0 = C3 * C2 * C1; % initial DCM for Omega matrix 


ASW = DefineAircraftGeometry();
J = ASW.RotationalInertiaMatrix;
Jxx = J(1,1);
Jxy = J(1,2);
Jxz = J(1,3);
Jyy = J(2,2);
Jyz = J(2,3);
Jzz = J(3,3);

% Problem 4
vert.stab = ASW.vert_stab;
disp(vert.stab);
disp(J);

% Problem 5
Rho = 1.225 ;

% C
surfaces = [ASW.hor_stab; ASW.vert_stab; ASW.left_wing; ASW.right_wing];

% Compute Lift Curve Slope for all four surfaces
Cla_s = zeros(4,1);
for i=1:length(surfaces)
    Cla_s(i) = LiftCurveSlope(surfaces(i));
end



function CLa_s = LiftCurveSlope(surface)
AR = surface.aspect_ratio;
CLa_s = 2*pi*(AR/(2 + AR));
end


% This function would be used as a simulink block to compute
% localangleofattack.
function a_s = LocalAngleofAttack(surface,V_BA)
num_term = dot(V_BA, surface.normal);
den_term = norm(V_BA);
div = num_term / den_term;
a_s = surface.i - asin(div);
end


