close all;

% Set the default text interpreter to latex
set(groot, 'defaultTextInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');

%%

folder = cd;
folder = fullfile(folder, '..');
folder = fullfile(folder, '\simulink');
v = version('-release');
model_name = ['Pacejka_Tyre_Model_' v];
folder = fullfile(folder, ['\' model_name '.slx']);

open(folder);

mu = [0.1, 0.3, 0.7, 0.9];
delta_s = 0.001;
slip = (0:delta_s:1)';
alpha = slip*80;
t = slip;
ONE = ones(size(t));
Fz = 4*ONE;
slip = slip*100;

% clearing placeholder values
ds = createInputDataset(model_name);
% Turning on external input
set_param(model_name,'LoadExternalInput','on');

% Set the simulation stop time
set_param(model_name, 'StopTime', num2str(1));  % Set stop time to 10 seconds

% Set the fixed step size
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'FixedStep', num2str(delta_s));  % Set step size to 0.01 seconds



% loading the Fz:
ds{1} = ds{1}.delsample('Index',[1,2]);
ds{1} = ds{1}.addsample('time',t,'data',Fz);

% Initialisation: 
ds{2} = ds{2}.delsample('Index',[1,2]);
ds{2} = ds{2}.addsample('time',t,'data',Fz);
ds{3} = ds{3}.delsample('Index',[1,2]);
ds{3} = ds{3}.addsample('time',t,'data',Fz);
ds{4} = ds{4}.delsample('Index',[1,2]);
ds{4} = ds{4}.addsample('time',t,'data',Fz);

disp(ds)

figure(1);
hold all;
ylabel('$F_x$ [N]');
xlabel('$s$ [\%]');

figure(2);
hold all;
xlabel('$\alpha$ [deg]');
ylabel('$F_y$ [N]');

% 
% Initialize cell array for legend labels
legend_labels = cell(length(mu), 1);

i = 1;
for mu_val = mu 
    % loading the friction
    ds{3} = ds{3}.delsample('Index',1:length(t));
    ds{3} = ds{3}.addsample('time', t,'data', mu_val*ONE);
    % setting the slip to 0
    ds{2} = ds{2}.delsample('Index',1:length(t));
    ds{2} = ds{2}.addsample('time', t,'data', 0*ONE);
    % loading the alpha
    ds{4} = ds{4}.delsample('Index',1:length(t));
    ds{4} = ds{4}.addsample('time', t,'data', alpha);


    set_param(model_name,'ExternalInput','ds');

    simout = sim(model_name);

    Fy = simout.yout.get('Fy').Values.Data;

    figure(2);
    plot(alpha, Fy);


    % loading the slip
    ds{2} = ds{2}.delsample('Index',1:length(t));
    ds{2} = ds{2}.addsample('time', t,'data', slip);
    % setting the alpha to 0
    ds{4} = ds{4}.delsample('Index',1:length(t));
    
    ds{4} = ds{4}.addsample('time', t,'data', 0*alpha);
    set_param(model_name,'ExternalInput', 'ds');

    simout = sim(model_name);

    Fx = simout.yout.get('Fx').Values.Data;

    figure(1);
    plot(slip, Fx);

    legend_labels{i} = ['$\mu$ = ', num2str(mu(i))];
    i = i + 1;
end

save_path = cd;
save_path = fullfile(save_path, '..');
save_path = fullfile(save_path, '..');
save_path = fullfile(save_path, '\LaTeX');
save_path = fullfile(save_path, '\figures');


f = figure(1);
% Add legend to figure
legend(legend_labels, 'Interpreter', 'latex');
grid on;

f.Name = 'Fx_slip';
set(gcf, 'Renderer', 'Painters');
if(SAVE)
    saveas(f,[save_path '\' f.Name '.eps'],'epsc');
end

f = figure(2);
% Add legend to figure
legend(legend_labels, 'Interpreter', 'latex');
grid on;

f.Name = 'Fy_alpha';
set(gcf, 'Renderer', 'Painters');
if(SAVE)
    saveas(f,[save_path '\' f.Name '.eps'],'epsc');
end

% close without saving
close_system(model_name, 0);

