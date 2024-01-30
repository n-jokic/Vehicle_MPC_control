% point stabilization + Multiple shooting + Runge Kutta
clear all
close all
clc

% CasADi v3.5.1
addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.5.1')
import casadi.*

h = 0.2; %[s]
N = 10; % prediction horizon
rob_diam = 0.3;

v_max = 0.6; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include the initial state and the reference state)

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + h/2*k1, con); % new
    k3 = f(st + h/2*k2, con); % new
    k4 = f(st + h*k3, con); % new
    st_next_RK4=st +h/6*(k1 +2*k2 +2*k3 +k4); % new    
    % f_value = f(st,con);
    % st_next_euler = st+ (h*f_value);
    % g = [g;st_next-st_next_euler]; % compute constraints
    g = [g;st_next-st_next_RK4]; % compute constraints % new
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:3:3*(N+1),1) = -2; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 2; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -2; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 2; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = [0 ; 0 ; 0.0];    % initial condition.
xs = [1.5 ; 1.5 ; 0.0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / h)
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(h, t0, x0, u,f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end;
main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2)
average_mpc_time = main_loop_time/(mpciter+1)

Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,rob_diam)
%% Model
%          model parameters:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81; % gravitational acceleration
m = 1; % [tonns]
I = 1; % car inertia
a = 2.5; % distance from CM to front tyres
b = 2.5; % distance from CM to back tyres
c = 2; % width of a car

r = 0.3; % diameter of a wheel 
Idf = 1; % Inertia of front tyres
Idr = 1; % Inerita of rear tyres
Bd = 1; % tyre angular speed damping factor

Fzf = b*mg/2/(a+b); %force on front tyres
Fzr = a*mg/2/(a+b); %force on rear tyres



%          model states:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ydot = casadi.SX.sym('ydot');
xdot = casadi.SX.sym('xdot');

psi = casadi.SX.sym('psi');
psidot = casadi.SX.sym('psidot');

Y = casadi.SX.sym('Y');
X = casadi.SX.sym('X');

wfl = casadi.SX.sym('wfl');
wfr = casadi.SX.sym('wfr');
wrl = casadi.SX.sym('wrl');
wrr = casadi.SX.sym('wrr');

%          state vector:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = [ydot; xdot;
     psi; psidot;
     Y; X;
     wfl; wfr; wrl; wrr
     ];

%          control states:      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deltaf = casadi.SX.sym('deltaf');

Tbfl = casadi.SX.sym('Tbfl');
Tbfr = casadi.SX.sym('Tbfr');
Tbrl = casadi.SX.sym('Tbrl');
Tbrr = casadi.SX.sym('Tbrr');

%          control vector:      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u = [deltaf;
     Tbfl; Tbfr; Tbrl; Tbrr
     ];

%       auxiliary states:       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vl = casadi.SX.sym('vl');
vc = casadi.SX.sym('vc');
w = casadi.SX.sym('w');
T = casadi.SX.sym('T');
s = casadi.SX.sym('s');

%       auxiliary functions:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

slip_driving = casadi.Function('slip_driving', {vl, w}, ...
    {if_else(w==0, ...
             if_else(vl==0, ...
                     1-1/r, ...
                     -sign(vl) ...
                     ), ...
             1-vl/w/r ...
            ) ...
    } ...
    , {'vl', 'w'}, {'s'});

slip_braking = casadi.Function('slip_braking', {vl, w}, ...
    {if_else(vl==0, ...
             if_else(w==0, ...
                     r-1, ...
                     sign(w) ...
                     ), ...
             r*w/vl - 1 ...
            ) ...
    } ...
    , {'vl', 'w'}, {'s'});

clip = casadi.Function('s', {s}, ...
                       {if_else(s>1, ...
                                1, ...
                                if_else(s<-1, ...
                                        -1, ...
                                        s))}...
                      , {'s'}, {'s_clip'});

slip = casadi.Function('slip', {vl, w, T}, ...
    {if_else(T > 0, ...
             100*clip(slip_driving(vl, w)), ...
             100*clip(slip_braking(vl, w)) ...
            ) ...
    } ...
    , {'vl', 'w', 'T'}, {'s'});


%          model equations:     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%

import casadi.*

T = 10; % Time horizon
N = 20; % number of control intervals

% Declare model variables
x1 = SX.sym('x1');
x2 = SX.sym('x2');
x = [x1; x2];
u = SX.sym('u');

% Model equations
xdot = [(1-x2^2)*x1 - x2 + u; x1];

% Objective term
L = x1^2 + x2^2 + u^2;

% Continuous time dynamics
f = Function('f', {x, u}, {xdot, L});

% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
M = 4; % RK4 steps per interval
DT = T/N/M;
X0 = MX.sym('X0', 2);
U = MX.sym('U');
X = X0;
Q = 0;
for j=1:M
   [k1, k1_q] = f(X, U);
   [k2, k2_q] = f(X + DT/2 * k1, U);
   [k3, k3_q] = f(X + DT/2 * k2, U);
   [k4, k4_q] = f(X + DT * k3, U);
   X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
   Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions
X0 = MX.sym('X0', 2);
w = {w{:}, X0};
lbw = [lbw; 0; 1];
ubw = [ubw; 0; 1];
w0 = [w0; 0; 1];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    w = {w{:}, Uk};
    lbw = [lbw; -1];
    ubw = [ubw;  1];
    w0 = [w0;  0];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], 2);
    w = {w{:}, Xk};
    lbw = [lbw; -0.25; -inf];
    ubw = [ubw;  inf;  inf];
    w0 = [w0; 0; 0];

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
solver = nlpsol('solver', 'ipopt', prob, options);

s0 = MX.sym('s0',2);

lbw_sym = MX(lbw);
ubw_sym = MX(ubw);
lbw_sym(1:2) = s0;
ubw_sym(1:2) = s0;

sol_sym = solver('x0', w0, 'lbx', lbw_sym, 'ubx', ubw_sym,...
            'lbg', lbg, 'ubg', ubg);

% Mapping from initial state to control action

function_name = 'f';
f = Function(function_name,{s0},{sol_sym.x(3)});

file_name = 'f.casadi';
f.save(file_name);

lib_path = GlobalOptions.getCasadiPath();
inc_path = GlobalOptions.getCasadiIncludePath();
mex('-v',['-I' inc_path],['-L' lib_path],'-lcasadi', 'casadi_fun.c')

%%
open_system('mpc_demo')

