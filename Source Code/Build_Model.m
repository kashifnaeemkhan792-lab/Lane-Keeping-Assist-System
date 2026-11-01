%% Build LKA Simulink Model Automatically
% This script creates the Simulink block diagram programmatically

model = 'LKA_Simulink_Model';

% Close if already open
if bdIsLoaded(model)
    close_system(model, 0);
end

% Create new model
new_system(model);
open_system(model);

%% Add Blocks

% 1. Constant block - Vehicle Speed
add_block('simulink/Sources/Constant', [model '/Vehicle Speed (v)']);
set_param([model '/Vehicle Speed (v)'], 'Value', '27.78', 'Position', [50 40 130 70]);

% 2. Constant block - Initial Lateral Error
add_block('simulink/Sources/Constant', [model '/Initial y0'], 'Value', '0.6', 'Position', [50 120 130 150]);

% 3. Constant block - Initial Heading Error  
add_block('simulink/Sources/Constant', [model '/Initial psi0'], 'Value', '0.0524', 'Position', [50 200 130 230]);

% 4. MATLAB Function - MPC Controller
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/MPC Controller']);
set_param([model '/MPC Controller'], 'Position', [350 100 500 200]);

% 5. MATLAB Function - Kinematic Bicycle Model (Plant)
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/Bicycle Model']);
set_param([model '/Bicycle Model'], 'Position', [600 100 750 200]);

% 6. MATLAB Function - LiDAR Sensor (with noise)
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/LiDAR Sensor']);
set_param([model '/LiDAR Sensor'], 'Position', [350 300 500 380]);

% 7. Scope - Lateral Position
add_block('simulink/Sinks/Scope', [model '/Lateral Position Scope']);
set_param([model '/Lateral Position Scope'], 'Position', [850 50 890 90]);

% 8. Scope - Heading Angle
add_block('simulink/Sinks/Scope', [model '/Heading Scope']);
set_param([model '/Heading Scope'], 'Position', [850 140 890 180]);

% 9. Scope - Steering Angle
add_block('simulink/Sinks/Scope', [model '/Steering Scope']);
set_param([model '/Steering Scope'], 'Position', [850 230 890 270]);

% 10. To Workspace blocks (for plotting later)
add_block('simulink/Sinks/To Workspace', [model '/y_out']);
set_param([model '/y_out'], 'VariableName', 'y_sim', 'Position', [850 300 930 330]);

add_block('simulink/Sinks/To Workspace', [model '/psi_out']);
set_param([model '/psi_out'], 'VariableName', 'psi_sim', 'Position', [850 360 930 390]);

%% Add Lines (Connections)
% Connect speed to MPC and Bicycle Model
add_line(model, 'Vehicle Speed (v)/1', 'MPC Controller/1', 'autoroute', 'on');

%% Add Annotations
add_block('simulink/Annotations/Note', [model '/Title']);

%% Set Simulation Parameters
set_param(model, 'StopTime', '10');
set_param(model, 'Solver', 'FixedStepDiscrete');
set_param(model, 'FixedStep', '0.1');

% Save model
save_system(model);

disp('Simulink model created successfully!')
disp('NOTE: You need to manually edit the MATLAB Function blocks.')
disp('Double-click each one to add the code - instructions coming next.')