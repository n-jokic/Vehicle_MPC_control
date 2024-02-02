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
Bd = 0.1; % tyre angular speed damping factor

Fzf = b*m*g/2/(a+b)/1000; % [kN] force on front tyres
Fzr = a*m*g/2/(a+b)/1000; % [kN] force on rear tyres