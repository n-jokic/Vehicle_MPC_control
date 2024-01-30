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

wfl = casadi.SX.sym('wfl');
wfr = casadi.SX.sym('wfr');
wrl = casadi.SX.sym('wrl');
wrr = casadi.SX.sym('wrr');

%          state vector:        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
states = [ydot; xdot;
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
control = [deltaf;
     Tbfl; Tbfr; Tbrl; Tbrr
     ];

%       auxiliary states:       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = casadi.SX.sym('w');
T = casadi.SX.sym('T');
s = casadi.SX.sym('s');
vx = casadi.SX.sym('vx');
vy = casadi.SX.sym('vy');
delta = casadi.SX.sym('delta');
angle = casadi.SX.sym('angle');
mu = casadi.SX.sym('mu');
Fx = casadi.SX.sym('Fx');
Fy = casadi.SX.sym('Fy');
slip_vector = casadi.SX.sym('slip_vector', 4); 
alpha_vector = casadi.SX.sym('alpha_vector', 4); 
forces = casadi.SX.sym('forces', 12);
delta_front = casadi.SX.sym('delta_front');
delta_rear = casadi.SX.sym('delta_rear');
vl_vct = casadi.SX.sym('vl_vct', 4); 
vc_vct = casadi.SX.sym('vc_vct', 4); 
vl_vc = casadi.SX.sym('vl_vc_vct', 8);

%  wheel speeds calculation:    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vyfl = casadi.Function('vyfl', {ydot, psidot}, ...
                        {ydot + a*psidot} ...
                        ,{'ydot', 'psidot'}, {'vyfl'});
vxfl = casadi.Function('vxfl', {xdot, psidot}, ...
                        {xdot - c*psidot} ...
                        ,{'xdot', 'psidot'}, {'vxfl'});

vyfr = casadi.Function('vyfr', {ydot, psidot}, ...
                        {ydot + a*psidot} ...
                        ,{'ydot', 'psidot'}, {'vyfr'});
vxfr = casadi.Function('vxfr', {xdot, psidot}, ...
                        {xdot + c*psidot} ...
                        ,{'xdot', 'psidot'}, {'vxfr'});

vyrl = casadi.Function('vyrl', {ydot, psidot}, ...
                        {ydot - b*psidot} ...
                        ,{'ydot', 'psidot'}, {'vyrl'});
vxrl = casadi.Function('vxrl', {xdot, psidot}, ...
                        {xdot - c*psidot} ...
                        ,{'xdot', 'psidot'}, {'vxrl'});

vyrr = casadi.Function('vyrr', {ydot, psidot}, ...
                        {ydot - b*psidot} ...
                        ,{'ydot', 'psidot'}, {'vyrr'});
vxrr = casadi.Function('vxfl', {xdot, psidot}, ...
                        {xdot + c*psidot} ...
                        ,{'xdot', 'psidot'}, {'vxrr'});

v_vctr = casadi.Function('v_vctr', {ydot, xdot, psidot}, ...
                        {[ ...
                        vxfl(xdot, psidot); ...
                        vyfl(ydot, psidot); ...
                        vxfr(xdot, psidot); ...
                        vyfr(ydot, psidot); ...
                        vxrl(xdot, psidot); ...
                        vyrl(ydot, psidot); ...
                        vxrr(xdot, psidot); ...
                        vyrr(ydot, psidot); ...
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

mixing_matrix = casadi.Function('mixing_matrix', ...
    {delta_front, delta_rear}, ...
    {[[ cos(delta_front), sin(delta_front), 0, 0, 0, 0, 0, 0]; ...
      [-sin(delta_front), cos(delta_front), 0, 0, 0, 0, 0, 0];...
      [ 0, 0, cos(delta_front), sin(delta_front), 0, 0, 0, 0]; ...
      [ 0, 0,-sin(delta_front), cos(delta_front), 0, 0, 0, 0];...
      [ 0, 0, 0, 0, cos(delta_rear), sin(delta_rear), 0, 0]; ...
      [ 0, 0, 0, 0,-sin(delta_rear), cos(delta_rear), 0, 0];...
      [ 0, 0, 0, 0, 0, 0, cos(delta_rear), sin(delta_rear)]; ...
      [ 0, 0, 0, 0, 0, 0,-sin(delta_rear), cos(delta_rear)];...
    ] ...
    }, {'delta_front', 'delta_rear'}, {'mixing_matrix'});

vl_vc_vctr = casadi.Function('vl_vc_vctr', ...
    {ydot, xdot, psidot, delta_front, delta_rear}, ...
    {mixing_matrix(delta_front, delta_rear)*v_vctr(ydot, xdot, psidot)}...
    , {'ydot', 'xdot', 'psidot', 'delta_front', 'delta_rear'}, ...
    {'vl_vc_vctr'});

%  slip and alpha calculation:  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

slip_driving = casadi.Function('slip_driving', {vx, w}, ...
    {if_else(w==0, ...
             if_else(vx==0, ...
                     1-1/r, ...
                     -sign(vx) ...
                     ), ...
             1-vx/w/r ...
            ) ...
    } ...
    , {'vl', 'w'}, {'s'});

slip_braking = casadi.Function('slip_braking', {vx, w}, ...
    {if_else(vx==0, ...
             if_else(w==0, ...
                     r-1, ...
                     sign(w) ...
                     ), ...
             r*w/vx - 1 ...
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

slip = casadi.Function('slip', {vx, w, T}, ...
    {if_else(T > 0, ...
             100*clip(slip_braking(vx, w)), ...
             100*clip(slip_driving(vx, w)) ...
            ) ...
    } ...
    , {'vl', 'w', 'T'}, {'s'});


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
    {[ -states(2)*states(4) + sum(forces(2:2:end-4))/m;...
       states(1)*states(4) + sum(forces(1:2:end-4))/m;...
       states(4);
       1/I*(a*(forces(2)+forces(4)) ...
           - b*(forces(6)+forces(8)) ...
           +c*(-forces(1)+forces(3)-forces(5)+forces(7)) ...
           ); ...
       states(2)*sin(states(3)) + states(1)*cos(states(3));...
       states(2)*cos(states(3)) + states(1)*sin(states(3));...
       1/Idf*(-forces(end-3)*r-control(2)-Bd*states(7));...
       1/Idf*(-forces(end-2)*r-control(3)-Bd*states(8));...
       1/Idr*(-forces(end-1)*r-control(4)-Bd*states(9));...
       1/Idr*(-forces(end)*r-control(5)-Bd*states(10));...
    ] ...
    } ...
    , {'states', 'control', 'forces'}, {'dot_states_force_wrapper'});

state_transition_slip_alpha = casadi.Function('f_slip_alpha', ...
    {states, control, slip_vector, alpha_vector}, ...
    {state_transition_force(states, control, ...
    [Fx_Fy(control(1), alpha_vector(1), slip_vector(1), friction); ...
     Fx_Fy(control(1), alpha_vector(2), slip_vector(2), friction); ...
     Fx_Fy(0, alpha_vector(3), slip_vector(3), friction); ...
     Fx_Fy(0, alpha_vector(4), slip_vector(4), friction); ...
     [1, 0]*Fl_Fc(Fl0(slip_vector(1), friction),... 
                  Fc0(alpha_vector(1), friction), ...
                  beta(alpha_vector(1), slip_vector(1)) ...
                  ); ...
     [1, 0]*Fl_Fc(Fl0(slip_vector(1), friction),... 
                  Fc0(alpha_vector(1), friction), ...
                  beta(alpha_vector(1), slip_vector(1)) ...
                  ); ...
     [1, 0]*Fl_Fc(Fl0(slip_vector(1), friction),... 
                  Fc0(alpha_vector(1), friction), ...
                  beta(alpha_vector(1), slip_vector(1)) ...
                  ); ...
     [1, 0]*Fl_Fc(Fl0(slip_vector(1), friction),... 
                  Fc0(alpha_vector(1), friction), ...
                  beta(alpha_vector(1), slip_vector(1)) ...
                  ) ...
    ] ...
    ) ...
    } ...
    , {'states', 'control', 'slip_vector', 'alpha_vector'}, ...
    {'dot_states_alpha_slip_wrapper'});


state_transition_speed = casadi.Function('f_speed', ...
    {states, control, vl_vc},...
    {state_transition_slip_alpha(states, control, ...
    slip(vl_vc(1:2:end), states(7:10), control(2:5)), ...
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



