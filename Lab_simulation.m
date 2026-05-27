close all
clear
clc

global config
global prev_T_target
% robot = importrobot('abbIrb1600.urdf');
robot = loadrobot('abbIrb1600', 'DataFormat', 'struct');
config = robot.homeConfiguration;
tform = getTransform(robot,config,"tool0");
prev_T_target = tform;

% ----- Add frames -----
robot = addFrame([-105.513,2.40649,246.356],[1,0,0,0],robot,'t4','t4j','tool0');
robot = addFrame([559.804,5.50957,-3.63248],[0.999987,-0.00156359,-0.00487101,7.47128E-05],robot,'uframe','uframej','base_link');
robot = addFrame([5,4,0],[0.67559,0,0,-0.737277],robot,'oframe','oframej','uframe');
robot = addFrame([-46.86,-7.90,235.64],[0.0498083,-0.0133606,-0.998594,-0.0123139],robot,'p10','p10j','oframe');
robot = addFrame([-21.39,-34.91,-0.63],[0.0497861,-0.0134405,-0.998589,-0.0127018],robot,'p20','p20j','oframe');
robot = addFrame([19.16,-34.92,-0.53],[0.0498128,-0.0133747,-0.998593,-0.0123192],robot,'p30','p30j','oframe');
robot = addFrame([19.17,35.63,-0.49],[0.0498143,-0.0133815,-0.998593,-0.0123221],robot,'p40','p40j','oframe');
robot = addFrame([-21.65,35.64,-0.22],[0.0498108,-0.0133915,-0.998593,-0.0123259],robot,'p50','p50j','oframe');
robot = addFrame([-21.65,35.64,-0.22],[0.0498136,-0.0133915,-0.998593,-0.0123257],robot,'p60','p60j','oframe');

% ----- Single figure + axes -----
fig = figure('Name','ABB Simulation');
ax  = axes('Parent', fig);

show(robot, config, 'Parent', ax, 'Visuals','on', 'Frames','off');
% show(robot, config, 'PreservePlot', false, 'Visuals', 'on', 'FastUpdate', false);
view(ax, 3);
grid(ax, 'on');
axis(ax, 'equal');

%new add 

% Camera position — pull back and elevate for a roomier feel
view(ax, [135 25]);          % azimuth 135°, elevation 25° — isometric-ish
camzoom(ax, 0.7);            % zoom out (< 1 = further away)

% Better lighting so the mesh doesn't look flat/dark
camlight(ax, 'headlight');
lighting(ax, 'gouraud');

% Tighter grid lines look cleaner at larger scales
ax.GridAlpha = 0.3;          % subtle grid
ax.GridLineStyle = '--';
ax.Color = [0.12 0.12 0.12]; % keep dark background

% Remove axis label clutter if you want a cleaner look
ax.XLabel.String = 'X (m)';
ax.YLabel.String = 'Y (m)';
ax.ZLabel.String = 'Z (m)';

% old
hold(ax, 'on');

% Fix workspace limits 
xlim(ax, [-0.5 1.5]);
ylim(ax, [-1.0 1.0]);
zlim(ax, [-0.3 1.8]);

% ----- Motions -----
MoveJ("p10",robot,'t4');
MoveJ("p20",robot,'t4');
MoveJ("p30",robot,'t4');
MoveJ("p40",robot,'t4');
MoveJ("p50",robot,'t4');
MoveJ("p60",robot,'t4');
MoveJ("p20",robot,'t4');
MoveJ("p10",robot,'t4');

%% ================= FUNCTIONS =================
function robot = addFrame(Trans, q, robot, name, jointname, parentname)
    R = quat2rotm(q);
    T = [[R; [0 0 0]], [(Trans./1000)'; 1]];  
    frame = rigidBody(name);
    jnt1 = rigidBodyJoint(jointname,'fixed');
    setFixedTransform(jnt1,T);
    frame.Joint = jnt1;
    addBody(robot, frame, parentname);
end

function MoveJ(endName, robot, toolName)
global config
global prev_T_target
    % Internal lookup using string names
    T_start  = prev_T_target;
    T_target = getTransform(robot,config,endName);
    % Setup Inverse Kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.25 0.25 0.25 1 1 1]; % Prioritize position (XYZ) over orientation
    initial_guess = config;

    % Solve IK for the start and end targets
    [q_start, ~] = ik(toolName, T_start, weights, initial_guess);
    [q_end,   ~] = ik(toolName, T_target, weights, q_start);

    % Interpolation parameters
    steps = 50;
    t = linspace(0, 1, steps);
    
    % Extract numeric values from the config structures
    start_vals = [q_start.JointPosition];
    end_vals = [q_end.JointPosition];
    
    % Animation Loop
    current_config = q_start; 
    for i = 1:steps
        % Linear interpolation in joint space
        current_vals = start_vals * (1 - t(i)) + end_vals * t(i);
        
        % Update structure efficiently
        for j = 1:length(current_vals)
            current_config(j).JointPosition = current_vals(j);
        end
        
        % Visualize 
        show(robot, current_config, 'PreservePlot', false, 'FastUpdate', true);
        drawnow limitrate
        pause(0.02);
    end
    config = current_config;
    prev_T_target = T_target;
end