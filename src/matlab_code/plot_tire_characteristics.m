close all;
clear; 

%%

folder = cd;
folder = fullfile(folder, '..');
folder = fullfile(folder, '\simulink');

model_name = 'Pacejka_Tyre_Model';
folder = fullfile(folder, ['\' model_name '.slx']);
open(folder);

% clearing placeholder values
ds = createInputDataset(model_name);
for i = 1 : length(ds)
    ds{i} = ds{i}.delsample('Index',[1,2]);

    % testing writing the values:
    ds{i} = ds{1}.addsample('time',[1 2]','data',[1 2]');
end

% Turning on external input
set_param(model_name,'LoadExternalInput','on');
%Setting param values
set_param(model_name,'ExternalInput','ds');

out = sim(folder);

