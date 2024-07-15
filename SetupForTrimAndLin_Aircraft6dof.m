clear all
clc

%% Define Simulation Parameters
startTime = 0;
stopTime = inf;
stepTime = 0.01;

RatioToRealTime = .8;

%% Define Universal Constants
deg2rad = pi/180;

%% Define Control Allocation Matrices
MixMatrix = [-1 1 0 0;
              1 1 0 0;
              0 0 1 0;
              0 0 0 1] ;

DeMixMatrix = inv(MixMatrix);

%% Define Actuator Initial Conditions and Parameters
dER_0 = 0; dEL_0 = 0; dE_0 = 0; dR_0 = 0;
tauER = 0.1; tauEL = 0.1; tauE = 0.1; tauR = 0.1;

%% Define Aircraft Mass and Geometry Properties (using grams)
%    mass   xSize   ySize   zSize   xLoc    yLoc    zLoc
%    1      2       3       4       5       6       7
componentMassesAndGeom = ...
    [90     0.1     0.48    0.01    -0.23   0.44    0;        % Wing01+Servo
     90     0.1	    0.48    0.01    -0.23   -0.44   0;        % Wing02+Servo
     13     0.075   0.35    0.002   -0.76   0       -0.16;     % Hor. Stab.
     0      0.08    0.002   0.18    -0.76   0       -0.09;    % Ver. Stab.
     72     0.065   0.035   0.015   -0.05   0       0.03;     % Battery
     106    0.87    0.07    0.07    -0.4    0       0;        % Fuselage
     27     0.05    0.03    0.005   -0.05   0       0.02;     % Motor Controller
     10     0.04    0.02    0.005   0.1     0       0.02;     % Radio
     20     0.05    0.01    0.01    -0.014  0       0;        % 2 Servos
     40     0.03    0.02    0.02    0.02    0       0.01;     % Motor
     12     0       0.26    0.025   0.05    0       0.01];    % Propeller

% Convert first column to kg
componentMassesAndGeom(:,1) = componentMassesAndGeom(:,1)/1000;
 
m = sum(componentMassesAndGeom(:,1));

componentMoments(:,1) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,5);
componentMoments(:,2) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,6);
componentMoments(:,3) = componentMassesAndGeom(:,1).*componentMassesAndGeom(:,7);

x_cm = sum(componentMoments)/m;
x_cm=x_cm';
%x_cm=x_cm+[.01;0;0];

for iComp=1:size(componentMassesAndGeom,1)
    if (componentMassesAndGeom(iComp,2) ~= 0)
        componentInertias(iComp,1) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,3).^2 + componentMassesAndGeom(iComp,4).^2);
        componentInertias(iComp,2) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,4).^2 + componentMassesAndGeom(iComp,2).^2);
        componentInertias(iComp,3) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,2).^2 + componentMassesAndGeom(iComp,3).^2);
    else
        componentInertias(iComp,1) = (1/12) * componentMassesAndGeom(iComp,1) .* (componentMassesAndGeom(iComp,3).^2 + componentMassesAndGeom(iComp,4).^2);        
    end
end

J=zeros(3,3);
J(1,1)=sum(componentInertias(:,1)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,6)-x_cm(2)).^2 + (componentMassesAndGeom(:,7)-x_cm(3)).^2));
J(2,2)=sum(componentInertias(:,2)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,7)-x_cm(3)).^2 + (componentMassesAndGeom(:,5)-x_cm(1)).^2));
J(3,3)=sum(componentInertias(:,3)) + sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,5)-x_cm(1)).^2 + (componentMassesAndGeom(:,6)-x_cm(2)).^2));
J(1,2) = -sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,5)-x_cm(1)) .* (componentMassesAndGeom(:,6)-x_cm(2))));
J(1,3) = -sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,7)-x_cm(3)) .* (componentMassesAndGeom(:,5)-x_cm(1))));
J(2,3) = -sum(componentMassesAndGeom(:,1) .* ((componentMassesAndGeom(:,6)-x_cm(2)) .* (componentMassesAndGeom(:,7)-x_cm(3))));

J(1,2) = -J(1,2);
J(1,3) = -J(1,3);
J(2,3) = -J(2,3);
J(2,1) = J(1,2);
J(3,1) = J(1,3);
J(3,2) = J(2,3);

%% Define Aircraft Aero Properties
% HorStab   = surface2 = s2
% VerStab   = surface3 = s3
% RightWing = surface4 = s4
% LeftWing  = surface5 = s5
x_s2 = [componentMassesAndGeom(3,5) + (1/4)*componentMassesAndGeom(3,2); componentMassesAndGeom(3,6);componentMassesAndGeom(3,7)];
x_s3 = [componentMassesAndGeom(4,5) + (1/4)*componentMassesAndGeom(4,2); componentMassesAndGeom(4,6);componentMassesAndGeom(4,7)]; %[-0.76 + (1/4)*0.08; 0; -0.09];
x_s4 = [componentMassesAndGeom(1,5) + (1/4)*componentMassesAndGeom(1,2); componentMassesAndGeom(1,6);componentMassesAndGeom(1,7)];
x_s5 = [componentMassesAndGeom(2,5) + (1/4)*componentMassesAndGeom(2,2); componentMassesAndGeom(2,6);componentMassesAndGeom(2,7)];

n_s2 = [0; 0; -1];
n_s3 = [0; 1; 0];
n_s4 = [0; sin(0*pi/180);  -cos(0*pi/180)];
n_s5 = [0; sin(0*pi/180);  -cos(0*pi/180)];
%  n_s4 = [0; sin( 1*pi/180);  -cos( 1*pi/180)];
%  n_s5 = [0; sin(-1*pi/180);  -cos(-1*pi/180)];

c_s2 = componentMassesAndGeom(3,2);
c_s3 = componentMassesAndGeom(4,2); %0.08;
c_s4 = componentMassesAndGeom(1,2);
c_s5 = componentMassesAndGeom(2,2);

c_u_s2 = 0.2*c_s2;
c_u_s3 = 0.2*c_s3;
c_u_s4 = 0.2*c_s4;
c_u_s5 = 0.2*c_s5;

b_s2 = componentMassesAndGeom(3,3);
b_s3 = componentMassesAndGeom(4,4); %0.08; <- Typo, should have been 0.18
b_s4 = componentMassesAndGeom(1,3);
b_s5 = componentMassesAndGeom(2,3);

S_s2 = c_s2 * b_s2;
S_s3 = c_s3 * b_s3;
S_s4 = c_s4 * b_s4;
S_s5 = c_s5 * b_s5;

% Full span control surfaces
S_u_s2 = c_u_s2*b_s2;
S_u_s3 = c_u_s3*b_s3;
S_u_s4 = c_u_s4*b_s4;
S_u_s5 = c_u_s5*b_s5;

AR_s2 = b_s2 / c_s2;
AR_s3 = b_s3 / c_s3;
AR_s4 = (b_s4+b_s5) / c_s4;
AR_s5 = (b_s4+b_s5) / c_s5;

CL0_s2 = 0;
CL0_s3 = 0;
CL0_s4 = 0.05;
CL0_s5 = 0.05;

e_s2 = 0.8;
e_s3 = 0.8;
e_s4 = 0.9;
e_s5 = 0.9;

i_s2 = -0.1;
i_s3 = 0;
i_s4 = 0.0;
i_s5 = 0.0;

CD0_s2 = 0.01;
CD0_s3 = 0.01;
CD0_s4 = 0.01;
CD0_s5 = 0.01;

CDa_s2 = 1;
CDa_s3 = 1;
CDa_s4 = 1;
CDa_s5 = 1;

a0_s2 = 0;
a0_s3 = 0;
a0_s4 = 0.05;
a0_s5 = 0.05;

CM0_s2 = 0;
CM0_s3 = 0;
CM0_s4 = -0.05;
CM0_s5 = -0.05;

CMa_s2 = 0;
CMa_s3 = 0;
CMa_s4 = 0;
CMa_s5 = 0;

dCL_du = 3; %lift coefficient increase per surface deflection in radians

%% Define Aircraft Propulsion system
R_p1 = componentMassesAndGeom(11,3)/2;
n_p1 = [1; 0; 0];
n_int(:,1) = n_p1;
x_p1 = componentMassesAndGeom(11,5:7)';

ARvecUIUCp1 = [0   ; 0.4 ; 1.0; 1.6  ];
CTvecUIUCp1 = [0.1 ; 0.09; 0  ; -0.09];
CPvecUIUCp1 = [0.05; 0.05; 0  ; -0.05];

ARvecP1 = ARvecUIUCp1 * 30/pi * 2;
CTvecP1 = CTvecUIUCp1 * 2 * (1/(2*pi))^2 * (2^4);
CPvecP1 = CPvecUIUCp1 * 2 * (1/(2*pi))^3 * (2^5);

CQvecP1 = CPvecP1;

% The first rotational internal component is the motor
J_int(1) = 0.01;
omDot_motor = 0;
om_motor_init = 0;

%% Define Motor Parameters
Rmotor=0.4; %Ohms
Io_motor=1.6; %Amps
Ki_motor=0.009549; %N.m/Amp


%% Define Battery Parameters
nCellsSeries = 6; % cells
VcellMax=4.2; %Volts
%Rcell=0.3; %Ohms
%nPacksParallel = inf;
%SocVec=[-0.5;0;0.1;0.75;1.0];
%dVoltageVec=[VcellMax;1.2;0.6;0.4;0.0];

%% Define Earth/Atmosphere Parameters
Rearth=6371000;
g_E_N=[0; 0; 9.81];
v_N_AE=[0; 0; 0];

%% Define Initial Conditions
% Initial p_E_E_BE=[lat; lon; alt] 
InitPos_deg = [37.6286; -122.393; 10]; % San Fransisco Airport
GoogleProvideLatLonAltOfUsBankInLa = [ 34.051122; -118.254399; 310 ];
GuessAndCheckToGetPlaneOnUsBankInLa = [34.05105; -118.25439; 415];
FlightGearOffset = GuessAndCheckToGetPlaneOnUsBankInLa - GoogleProvideLatLonAltOfUsBankInLa;

InitPos_deg = GoogleProvideLatLonAltOfUsBankInLa;
InitPos_deg = GoogleProvideLatLonAltOfUsBankInLa + FlightGearOffset;

InitPos = InitPos_deg .* [deg2rad; deg2rad; 1];
% Initial x_E_E_BE=[x; y; z] derives from p_E_E_BE
%x_E_E_BE0 = [Rearth;0;0];
x_E_E_BE0 = (Rearth+InitPos(3))*[cos(InitPos(1))*cos(InitPos(2)); ...
                    cos(InitPos(1))*sin(InitPos(2)); ...
                    sin(InitPos(1))];

% Initial att_E_E_BE=[roll; pitch; yaw]=[phi; theta; psi]
InitAtt_deg = [10; 10; 220];
InitAtt_deg = [0; 0; 220];
InitAtt_deg = [0; 0; 0];
InitAtt_deg = [0; -0.08*180/pi; 0];
InitAtt_deg = [0; 0; 0];
InitAtt = InitAtt_deg * deg2rad;

% Initial q_E_E_BE=[q0; q1; q2; a3] derives from att_E_E_BE
InitQuat(1) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(2) = sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(3) = cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(4) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2);
InitQuat(1)= 0.9996; InitQuat(2)=0; InitQuat(3)=-0.0409; InitQuat(4)=0;
InitQuat(1)= 1; InitQuat(2)=0; InitQuat(3)=0; InitQuat(4)=0;

% Initial v_B_B_BE=[u; v; w] 
InitVel = [13; 0; 0]; % With transform below, this is in NED axes
InitVel = [13; 0.25; 0.25]; % With transform below, this is in NED axes
InitVel = [13; 0; -0.25]; % With transform below, this is in NED axes
InitVel = [14.15; 0; -0.25]; % With transform below, this is in NED axes
InitVel = [14.1465; 0; -0.2535]; % With transform below, this is in NED axes
InitVel = [14.1897; 0; 0]; % With transform below, this is in NED axes
InitVel = [14.1897; 0; -0.2542]; % With transform below, this is in NED axes
InitVel = [15; 0; 0]; % With transform below, this is in NED axes
% InitVel = [cos(InitAtt(2)) 0 -sin(InitAtt(2));
%            0               1 0              ;
%            sin(InitAtt(2)) 0 cos(InitAtt(2))]*InitVel;

% Initial om_B_B_BE=[P; Q; R] 
InitRates = [1e-2; 1e-0; 1e-2];
InitRates = [1e-2; 1e-0; 1e-2];
InitRates = [0; 0; 0];


%% Define Forces Acting on the body in the NED Axes (easier to counter gravity this way)
Fext_N = -g_E_N * m + [0;1;0]*m;
Fext_N = -g_E_N * m;
Fext_N = [0;0;0];

%% Define Moments Acting on the body in the Body Axes
Mext_B = [.01; 0; 0];
Mext_B = [0; 0; 0];

