
SetupForTrimAndLin_Aircraft6dof;

[sizes,x0,names]=Aircraft6dof_forTrimAndLin_ATT_wInputs([],[],[],'sizes');

disp(names);

state_names = cell(1,numel(names));
for i = 1:numel(names)
  n = max(strfind(names{i},'/'));
  state_names{i}=names{i}(n+1:end);
end
disp(state_names);

desired_state_order     = [{'Phi'} {'Theta'} {'Psi'} {'U'} {'V'} {'W'} {'P'} {'Q'} {'R'} {'dER'} {'dEL'} {'dE'} {'dR'} {'hx'} {'hy'} {'hz'}];
fixed_states            = [{'Phi'} {'Psi'} {'V'} {'P'} {'Q'} {'R'} {'hx'} {'hy'} {'hz'} {'dER'} {'dEL'} {'dE'} {'dR'}];
fixed_derivatives       = [{'U'} {'W'} {'Q'}];        % U W and Q {'U'} {'W'} {'Q'}
fixed_outputs           = [];                     
fixed_inputs          = [1; 2; 3; 4]; %Right Elevon, Left Elevon, Elevator, Rudder

u0 = [0; 0; 0; 0];

n_states=[];
n_deriv=[];
states_order=[];

for i = 1:length(fixed_states)
  n_states=[n_states find(strcmp(fixed_states{i},state_names))];
end

for i = 1:length(desired_state_order)
  states_order=[states_order find(strcmp(desired_state_order{i},state_names))];
end
disp(states_order);

for i = 1:length(fixed_derivatives)
  n_deriv=[n_deriv find(strcmp(fixed_derivatives{i},state_names))]; 
end

trimOptionsIn = [1; 1e-4; 1e-4; 1e-6; 0;0;0;0;0;0;0;0;0; 2000; 0;1e-8;0.1;0];

[X_trim,U_trim,Y_trim,DX,trimOptions]=trim('Aircraft6dof_forTrimAndLin_ATT_wInputs',x0,u0,[], ...
                               n_states,fixed_inputs,[],[],n_deriv,trimOptionsIn);

Att_Trim=[X_trim(states_order(1)); X_trim(states_order(2)); X_trim(states_order(3))];        

Att_Trim_deg = Att_Trim * 180/pi;

Vel_B_BA_Trim = [X_trim(states_order(4)); X_trim(states_order(5)); X_trim(states_order(6))];
        
Rates_Trim = [X_trim(states_order(7)); X_trim(states_order(8)); X_trim(states_order(9))];

IntRates_Trim = [X_trim(states_order(10)); X_trim(states_order(11)); X_trim(states_order(12))];

Vel_W_Trim = [norm(Vel_B_BA_Trim); atan2(Vel_B_BA_Trim(3),Vel_B_BA_Trim(1)); asin(Vel_B_BA_Trim(2)/norm(Vel_B_BA_Trim)) ];

C_BN = Att2Dcm_fcn(Att_Trim);
C_NB = C_BN';

Vel_N_BA_Trim = C_NB * Vel_B_BA_Trim;


sys_struct = linmod('Aircraft6dof_forTrimAndLin_ATT_wInputs',X_trim);

sys_ss=ss(sys_struct.a,sys_struct.b,sys_struct.c,sys_struct.d);

sys_ss_wNoise=ss(sys_struct.a,[sys_struct.b eye(size(sys_struct.a,1))], ...
                 sys_struct.c,[sys_struct.d zeros(size(sys_struct.c,1),size(sys_struct.a,1))]);

sys_ss_minReal=minreal(sys_ss_wNoise);

%kalman(sys_ss_minReal,.00001*eye(size(sys_ss_minReal.a,1)),.01*eye(size(sys_ss_minReal.c,1)))

% But what is this? Kalman output will put out the state of the minReal
% system, which is not that useful. Redefine our system to not have so many
% states:

sys_ss_13states=ss([sys_struct.a(states_order(1):states_order(13), states_order(1):states_order(13))], ...
                 [sys_struct.b(states_order(1):states_order(13),:)], ...
                  [sys_struct.c(:,states_order(1):states_order(13))], ...
                 sys_struct.d);
sys_ss_13states_wNoise=ss(sys_ss_13states.a,[sys_ss_13states.b eye(size(sys_ss_13states.a,1))], ...
                 sys_ss_13states.c,[sys_ss_13states.d zeros(size(sys_ss_13states.c,1),size(sys_ss_13states.a,1))]);
% Now the system is a minimum realization
%sys_ss_minReal=minreal(sys_ss_wNoise)

[kest,L,P] = kalman(sys_ss_13states_wNoise,.01*eye(size(sys_ss_13states.a,1)),.01*eye(size(sys_ss_13states.c,1)));

[eigvecs, eigvals] = eig(sys_ss_13states.a);
real(eigvals);




