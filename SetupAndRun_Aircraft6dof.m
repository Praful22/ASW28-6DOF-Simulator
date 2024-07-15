%clear all
%clc

TrimAndLinearize_ATT_wInputs_Aircraft6dof;

%% Define Wind Disturbance
v_N_AE=[0; 0; 0];
g_E_N=[0; 0; 9.81]*1;


%% Define Out of Trim Initial Conditions (if Desired)
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
InitQuat(1)= 0.9996; InitQuat(2)=0; InitQuat(3)=-0.0409; InitQuat(4)=0;
InitAtt_deg = [10; 10; 220];
InitAtt_deg = [0; 0; 220];
InitAtt_deg = [0; -0.08*180/pi; 0];
InitAtt_deg = [0; 0; 0];
InitAtt_deg = Att_Trim_deg + [0;0;0];
InitAtt = InitAtt_deg * deg2rad;
% Initial q_E_E_BE=[q0; q1; q2; a3] derives from att_E_E_BE
InitQuat(1) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(2) = sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*cos(InitAtt(3)/2) - cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(3) = cos(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2) - sin(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2);
InitQuat(4) = cos(InitAtt(1)/2)*cos(InitAtt(2)/2)*sin(InitAtt(3)/2) - sin(InitAtt(1)/2)*sin(InitAtt(2)/2)*cos(InitAtt(3)/2);


% Initial v_B_B_BE=[u; v; w] 
InitVel = [13; 0; 0]; % With transform below, this is in NED axes
InitVel = [13; 0.25; 0.25]; % With transform below, this is in NED axes
InitVel = [13; 0; -0.25]; % With transform below, this is in NED axes
InitVel = [14.15; 0; -0.25]; % With transform below, this is in NED axes
InitVel = [14.1465; 0; -0.2535]; % With transform below, this is in NED axes
InitVel = [14.1897; 0; -0.2542]; % With transform below, this is in NED axes
InitVel = [0; 0; 0]; % With transform below, this is in NED axes
InitVel = Vel_B_BA_Trim;% - [6;0;0];% + [2;0;0] % With transform below, this is in NED axes
% InitVel = [cos(InitAtt(2)) 0 -sin(InitAtt(2));
%            0               1 0              ;
%            sin(InitAtt(2)) 0 cos(InitAtt(2))]*InitVel;

% Initial om_B_B_BE=[P; Q; R] 
InitRates = [1e-2; 1e-0; 1e-2];
InitRates = [1e-2; 1e-0; 1e-2];
InitRates = [0; 0; 0];
InitRates = [1e-0; -1e-2; -1e-2];
InitRates = Rates_Trim;


%% Define some arbitrary control surface deflections over time
time_vector_elevator =     [0  10 10.001 100   100.001 200   200.001 10000];
elevator_vector =          [0  0  0.1    0.1   -0.1    -0.1  0       0  ];

time_vector_rudder = [0  10 10.001 70   70.001 200   200.001 300   300.001 10000];
rudder_vector =      [0  0  -0.1   -0.1 0      0     0.1     0.1   0       0  ];

%% Define LQG estimator/regulator
%DesignLqgController;

%% Define LQG estimator/regulator
%DesignDecouplingController;

J = [0.049 0 -0.0004; 0 0.022 0; -0.0004 0 0.071];

%% Run the Simulation
sim('Aircraft6dof');

