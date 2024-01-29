addpath('C:\Users\Ivona\Desktop\CASADI\casadi-3.6.4-windows64-matlab2018b')
import casadi.*

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

delta_max = 10; %deg
delta_min = -10; %deg
M_max = 10^3;%Nm
M_min = -10^3; %Nm

%          model states:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ydot = SX.sym('ydot');
xdot = SX.sym('xdot');

psi = SX.sym('psi');
psidot = SX.sym('psidot');

Y = SX.sym('Y');
X = SX.sym('X');

%          state vector:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = [ydot; xdot;
     psi; psidot;
     Y; X];

%          control states:      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deltaf = SX.sym('deltaf');

M = SX.sym('M');


%          control vector:      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u = [deltaf;M];

%       auxiliary states:       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vl = SX.sym('vl');
vc = SX.sym('vc');
w = SX.sym('w');
T = SX.sym('T');
s = SX.sym('s');

%       auxiliary functions:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

slip_driving = Function('slip_driving', {vl, w}, ...
    {if_else(w==0, ...
             if_else(vl==0, ...
                     1-1/r, ...
                     -sign(vl) ...
                     ), ...
             1-vl/w/r ...
            ) ...
    } ...
    , {'vl', 'w'}, {'s'});

slip_braking = Function('slip_braking', {vl, w}, ...
    {if_else(vl==0, ...
             if_else(w==0, ...
                     r-1, ...
                     sign(w) ...
                     ), ...
             r*w/vl - 1 ...
            ) ...
    } ...
    , {'vl', 'w'}, {'s'});

clip = Function('s', {s}, ...
                       {if_else(s>1, ...
                                1, ...
                                if_else(s<-1, ...
                                        -1, ...
                                        s))}...
                      , {'s'}, {'s_clip'});

slip = Function('slip', {vl, w, T}, ...
    {if_else(T > 0, ...
             100*clip(slip_driving(vl, w)), ...
             100*clip(slip_braking(vl, w)) ...
            ) ...
    } ...
    , {'vl', 'w', 'T'}, {'s'});


%          model equations:     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% proracun za Fl, Fc
Flf = SX.sym('Flf');
Flr = SX.sym('Flr');
Fcf = SX.sym('Fcf');
Fcr = SX.sym('Fcr');

Fyf = Flf*sin(delta) + Fcf*cos(delta);
Fyr = Flr*sin(delta) + Fcr*cos(delta);
Fxf = Flf*cos(delta) - Flf*sin(delta);
Fxr = Flr*cos(delta) - Flr*sin(delta);

ydotdot = - xdot*psidot + 2/m*Fyf + 2/m*Fyr;
xdotdot = ydot*psidot + 2/m*Fxf + 2/m*Fxr; % ili xdotdot = 0, negde je ovako
psidotdot = 2*a/I*Fyf - 2*b/I*Fyr + M;
Xdot = xdot*cos(psi) - ydot*sin(psi);
Ydot = xdot*sin(psi) + ydot*cos(psi);

rhs = [xdotdot;ydotdot;psidot;psidotdot;Xdot;Ydot]; % sta sa psidot

L = delta^2 + M^2; % matrice S,Q?

% Continuous time dynamics
f = Function('f', {x, u}, {rhs, L});

% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator
Mk = 4; % RK4 steps per interval
DT = T/N/Mk;
X0 = MX.sym('X0', 6);
U = MX.sym('U',2);
X = X0;
Q = 0;
for j=1:Mk
   [k1, k1_q] = f(X, U);
   [k2, k2_q] = f(X + DT/2 * k1, U);
   [k3, k3_q] = f(X + DT/2 * k2, U);
   [k4, k4_q] = f(X + DT * k3, U);
   X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
   Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end

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
X0 = MX.sym('X0', 6);
w = {w{:}, X0};
lbw = [lbw; 0; 0; 0; 0; 0; 0];
ubw = [ubw; 0; 0; 0; 0; 0; 0];
w0 = [w0; 0; 0; 0; 0; 0; 0];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)],2);
    w = {w{:}, Uk};
    lbw = [lbw; delta_min; M_min];
    ubw = [ubw;  delta_max; M_max];
    w0 = [w0;  0;0];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], 2);
    w = {w{:}, Xk};
    lbw = [lbw; -inf; -inf;  -inf; -inf;  -inf; -inf];
    ubw = [ubw;  inf;  inf;inf;  inf;inf;  inf]; %dati realne ogranicenja
    w0 = [w0; 0; 0;0;0;0;0];

%     % Add equality constraint
%     g = {g{:}, Xk_end-Xk};
%     lbg = [lbg; 0; 0];
%     ubg = [ubg; 0; 0];
end