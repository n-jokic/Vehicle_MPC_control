%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%The values for the Michelin tyre MXV8%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The factors of the lateral force

disp('loading Michelin tyre MXV8');

a0 = 1.6;
a1 = -34;
a2 = 1250;
a3 = 2320;
a4 = 12.8;
a5 = 0;
a6 = -0.0053;
a7 = 0.1925;
a8 = 0;

% The factors of the longitudinal force

b0  = 1.55;
b1 = 0;
b2 = 1000; 
b3 = 60;
b4 = 300;
b5 = 0.17;
b6 = 0;
b7 = 0;
b8 = 0.2;

save('pacajeka_model_param');
