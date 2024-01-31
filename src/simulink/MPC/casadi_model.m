addpath('C:\Users\Ivona\Desktop\CASADI\casadi-3.6.4-windows64-matlab2018b')
%import casadi.*

%% Model
%          model parameters:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81; % gravitational acceleration
m = 1900; % [kg]
a = 1.05; % distance from CM to front tyres
b = 1.8-a; % distance from CM to back tyres
I = m*a*b; % car inertia
c = 1.5; % width of wheel axels
friction = 0.9; %friction coeff

slip = [0;0]; % slip = 0
r = 0.3284; % diameter of a wheel 
Idf = 0.805; % Inertia of front tyres
Idr = 0.805; % Inerita of rear tyres
Bd = 0.05; % tyre angular speed damping factor

Fzf = b*m*g/2/(a+b)/1000; % [kN] force on front tyres
Fzr = a*m*g/2/(a+b)/1000; % [kN] force on rear tyres



%          model states:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ydot = casadi.SX.sym('ydot');
xdot = casadi.SX.sym('xdot');

psi = casadi.SX.sym('psi');
psidot = casadi.SX.sym('psidot');

Y = casadi.SX.sym('Y');
X = casadi.SX.sym('X');

%          state vector:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
states = [ydot; xdot;
     psi; psidot;
     Y; X];

%          control states:      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deltaf = casadi.SX.sym('deltaf');

M = casadi.SX.sym('M');


%          control vector:      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
control = [deltaf;M];

%       auxiliary states:       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% w = casadi.SX.sym('w');
% T = casadi.SX.sym('T');

s = casadi.SX.sym('s'); % OVO JE 0
vx = casadi.SX.sym('vx');
vy = casadi.SX.sym('vy');
delta = casadi.SX.sym('delta');
angle = casadi.SX.sym('angle');
mu = casadi.SX.sym('mu');
Fx = casadi.SX.sym('Fx');
Fy = casadi.SX.sym('Fy');
slip_vector = casadi.SX.sym('slip_vector', 2);  % [0;0]
alpha_vector = casadi.SX.sym('alpha_vector', 2); 
forces = casadi.SX.sym('forces', 4);
delta_front = casadi.SX.sym('delta_front');
delta_rear = casadi.SX.sym('delta_rear'); % OVO JE 0
vl_vct = casadi.SX.sym('vl_vct', 2); 
vc_vct = casadi.SX.sym('vc_vct', 2); 
vl_vc = casadi.SX.sym('vl_vc_vct', 4);

%  wheel speeds calculation:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vyf = casadi.Function('vyf', {ydot, psidot}, ...
                        {ydot + a*psidot} ...
                        ,{'ydot', 'psidot'}, {'vyf'});
vxf = casadi.Function('vxf', {xdot}, ...
                        {xdot} ...
                        ,{'xdot'}, {'vxf'});
vyr = casadi.Function('vyr', {ydot, psidot}, ...
                        {ydot - b*psidot} ...
                        ,{'ydot', 'psidot'}, {'vyr'});
vxr = casadi.Function('vxr', {xdot}, ...
                        {xdot} ...
                        ,{'xdot'}, {'vxr'});

v_vctr = casadi.Function('v_vctr', {ydot, xdot, psidot}, ...
                        {[ ...
                        vxf(xdot); ...
                        vyf(ydot, psidot); ...
                        vxr(xdot); ...
                        vyr(ydot, psidot); ...
                        ]} ...
                        ,{'ydot', 'xdot', 'psidot'}, {'v_vctr'});
% longitudal/normal conversion: %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vl = casadi.Function('vl', {vy, vx, delta}, ...
                      {vy*sin(delta) + vx*cos(delta)} ...
                      , {'vy', 'vx', 'delta'}, {'vl'});

vc = casadi.Function('vc', {vy, vx, delta}, ...
                      {vy*sin(delta) - vx*cos(delta)} ...
                      , {'vy', 'vx', 'delta'}, {'vc'});

      
% delta_rear =0!

mixing_matrix = casadi.Function('mixing_matrix', ...
    {delta_front, delta_rear}, ...
    {[[ cos(delta_front), sin(delta_front), 0, 0]; ...
      [-sin(delta_front), cos(delta_front), 0, 0];...
      [ 0, 0, cos(delta_rear), sin(delta_rear)]; ...
      [ 0, 0,-sin(delta_rear), cos(delta_rear)];...
    ] ...
    }, {'delta_front', 'delta_rear'}, {'mixing_matrix'});

vl_vc_vctr = casadi.Function('vl_vc_vctr', ...
    {ydot, xdot, psidot, delta_front, delta_rear}, ...
    {mixing_matrix(delta_front, delta_rear)*v_vctr(ydot, xdot, psidot)}...
    , {'ydot', 'xdot', 'psidot', 'delta_front', 'delta_rear'}, ...
    {'vl_vc_vctr'});

%  slip and alpha calculation:  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

alpha = casadi.Function('alpha', {vy, vx}, ...
                        {atan2(vy, ...
                               vx)/pi*180}...
                        , {'vc', 'vl'}, {'alpha'});


%      Pajaceka Tyre model:     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C = b0;
D = (b1*Fzf^2 + b2*Fzf);
B = (b3*Fzf^2 + b4*Fzf)/(C*D)*exp(-b5*Fzf);
E = b6*Fzf^2 + b7*Fzf + b8;

Fl0 = casadi.Function('Fl0', {s, mu}, ...
                       {mu*D*sin(C*atan(B*(1-E)*s + E*atan(B*s)))} ...
                       , {'s', 'mu'}, {'Fl0'});

C = a0;
D = mu*(a1*Fzr^2 + a2*Fzr);
B = (a3*sin(a4*atan(Fzr/a5)))/C/D;
E = a6*Fzr^2 + a7*Fzr + a8;
Sh = 0;
Sv = 0;

Fc0 = casadi.Function('Fc0', {angle, mu}, ...
                       {-mu*D*sin(C*atan(B*(1-E)*(angle+Sh) + ...
                       E*atan(B*(angle+Sh)))) + Sv} ...
                       , {'alpha', 'mu'}, {'Fc0'});

beta = casadi.Function('beta', {angle, s}, ...
                        {atan2(sin(angle/180*pi), s/100)} ...
                        ,{'alpha', 's'}, {'beta'});

Fl_Fc = casadi.Function('Fl_Fc', {Fx, Fy, angle}, ...
                        {[Fx*abs(cos(angle)); ...
                          Fy*abs(sin(angle)) ...
                          ]} ...
                          , {'Fl', 'Fc', 'beta'}...
                          , {'Fl_Fc_vector'}...
                          );

Fx_Fy = casadi.Function('Fx_Fy', {delta, angle, s, mu}, ...
                         {[[cos(delta), -sin(delta)]; ...
                           [sin(delta), +cos(delta) ...
                           ]]*Fl_Fc(Fl0(s, mu), ...
                                Fc0(angle, mu), ...
                                beta(angle, s) ...
                                ) ...
                          } ...
                          , {'delta', 'alpha', 's', 'mu'}...
                          , {'Fx_Fy_vector'}...
                          );


%          model equations:     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_transition_force = casadi.Function('f_force', ...
    {states, control, forces}, ...
    {[ -states(2)*states(4) + 2*(forces(2) + forces(4))/m;...
       states(1)*states(4) + 2*(forces(1) + forces(3))/m;...
       states(4);
       1/I*(2*a*forces(2) - 2*b*forces(4) + control(1)); ...
       states(2)*sin(states(3)) + states(1)*cos(states(3));...
       states(2)*cos(states(3)) - states(1)*sin(states(3));...
    ] ...
    } ...
    , {'states', 'control', 'forces'}, {'dot_states_force_wrapper'});

% slip = [0,0]; !
state_transition_slip_alpha = casadi.Function('f_slip_alpha', ...
    {states, control, alpha_vector}, ...
    {state_transition_force(states, control, ...
    [Fx_Fy(control(1), alpha_vector(1), slip(1), friction); ...
     Fx_Fy(0, alpha_vector(2), slip(2), friction); ...
    ] ...
    ) ...
    } ...
    , {'states', 'control', 'alpha_vector'}, ...
    {'dot_states_alpha_slip_wrapper'});


state_transition_speed = casadi.Function('f_speed', ...
    {states, control, vl_vc},...
    {state_transition_slip_alpha(states, control, ... 
    alpha(vl_vc(2:2:end), vl_vc(1:2:end))...
    )...
    }...
    , {'states', 'control', 'vl_vc_vector'},...
    {'dot_states_speed_wrapper'}...
    );

state_transition = casadi.Function('f', ...
    {states, control},...
    {state_transition_speed(states, control, ...
    vl_vc_vctr(ydot, xdot, psidot, control(1)/180*pi, 0)...
    )...
    }...
    , {'states', 'control'},...
    {'dot_states'}...
    );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
delta_max = 10; %deg
delta_min = -10; %deg
M_max = 10^3;%Nm
M_min = -10^3; %Nm

L = control(1)^2 + control(2)^2; % matrice S,Q?



% Continuous time dynamics
f1 = casadi.Function('f1', {states, control}, {state_transition(states, control),L}, ...
    {'states', 'control'}, {'xdot','L'});
%%
N = 20;
T = 10;
% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
Mk = 4; % RK4 steps per interval
DT = T/N/Mk;
X0 = casadi.MX.sym('X0', 6);
U = casadi.MX.sym('U', 2);
X = X0;
Q = 0;
for j=1:Mk
    [k1, k1_q] = f1(X, U);
    [k2, k2_q] = f1(X + DT/2 * k1, U);
    [k3, k3_q] = f1(X + DT/2 * k2, U);
    [k4, k4_q] = f1(X + DT * k3, U);
    X = X + DT/6 * (k1 + 2*k2 + 2*k3 + k4);
    Q = Q + DT/6 * (k1_q + 2*k2_q + 2*k3_q + k4_q);
end

F = casadi.Function('F', {X0, U}, {X, Q}, {'x0', 'p'}, {'xf', 'qf'});
%%
% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions ?
X0 = casadi.MX.sym('X0', 6);
w = {w{:}, X0};
lbw = [lbw; -inf; -inf; -inf; -inf; -inf; -inf];
ubw = [ubw; inf; inf; inf; inf; inf; inf];
w0 = [w0; 0; 0; 0; 0; 0; 0];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk = casadi.MX.sym(['U_' num2str(k)],2);
    w = {w{:}, Uk};
    lbw = [lbw; delta_min; M_min];
    ubw = [ubw;  delta_max; M_max];
    w0 = [w0;  0;0];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = casadi.MX.sym(['X_' num2str(k+1)], 6);
    w = {w{:}, Xk};
    lbw = [lbw; -inf; -inf;  -inf; -inf;  -inf; -inf];
    ubw = [ubw;  inf;  inf;inf;  inf;inf;  inf]; %dati realne ogranicenja
    w0 = [w0; 0; 0;0;0;0;0];

    % Nedostaje deo za refrencu: 
%     % Add equality constraint 
%     g = {g{:}, Xk_end-Xk};
%     lbg = [lbg; 0; 0];
%     ubg = [ubg; 0; 0];
end



