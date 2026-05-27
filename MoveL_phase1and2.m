close all
clear
clc

% =========================================================
%  PHASE SELECTOR — change this to 1 or 2, then press F5
% =========================================================
PHASE = 2;
% =========================================================


% =========================================================
%  PHASE 1 — Simple test robot (test.urdf)
%  Proves MoveL interpolation + IK logic works
% =========================================================
if PHASE == 1

    robot = importrobot('test.urdf');
    showdetails(robot);

    % Create figure and axes
    fig = figure('Name', 'Phase 1 - MoveL Test');
    ax  = axes('Parent', fig);

    % Use homeConfig — not randomConfig — for a clean starting pose
    homeConfig = homeConfiguration(robot);
    show(robot, homeConfig, 'Parent', ax, 'Visuals', 'on', 'Frames', 'off');

    % Axis limits scaled for mm-unit URDF (joints span hundreds of units)
    xlim(ax, [-800  800]);
    ylim(ax, [-800  800]);
    zlim(ax, [-100 1500]);

    view(ax, [135 25]);
    grid(ax, 'on');
    axis(ax, 'manual');   %Prevent auto-rescaling during animation
    camzoom(ax, 0.8);
    hold(ax, 'on');

    % % Move from home to random target
    % home   = homeConfiguration(robot);
    % target = randomConfiguration(robot);
    
    % Move from home to specified target
    home   = homeConfiguration(robot);
    target = homeConfiguration(robot);
    target(1).JointPosition =  0.8;   % base rotates right
    target(2).JointPosition = -0.3;   % shoulder lifts up
    target(3).JointPosition = 0.4;   % elbow bends forward

    disp('Phase 1: Running MoveL from home to random target...');
    MoveL(home, target, robot, '', 30, ax);
    disp('Phase 1: Done!');

% =========================================================
%  PHASE 2 — Real ABB IRB1600 with RAPID coordinates
%  Mirrors the factory RAPID program path p10→p20→...→p10
% =========================================================
elseif PHASE == 2

    % Load the real ABB robot
    robot = loadrobot('abbIrb1600', 'DataFormat', 'struct');
    config = robot.homeConfiguration;

    % Add tool and work object frames (from RAPID tooldata/wobjdata)
    robot = addFrame([-105.513, 2.40649, 246.356],  [1,0,0,0],                                        robot,'t4',     't4j',     'tool0');
    robot = addFrame([559.804,  5.50957, -3.63248],  [0.999987,-0.00156359,-0.00487101,7.47128E-05],   robot,'uframe', 'uframej', 'base_link');
    robot = addFrame([5,        4,       0],          [0.67559,0,0,-0.737277],                          robot,'oframe', 'oframej', 'uframe');

    % Add RAPID robtargets as fixed frames
    robot = addFrame([-46.86, -7.90,  235.64], [0.0498083,-0.0133606,-0.998594,-0.0123139], robot,'p10','p10j','oframe');
    robot = addFrame([-21.39, -34.91, -0.63],  [0.0497861,-0.0134405,-0.998589,-0.0127018], robot,'p20','p20j','oframe');
    robot = addFrame([ 19.16, -34.92, -0.53],  [0.0498128,-0.0133747,-0.998593,-0.0123192], robot,'p30','p30j','oframe');
    robot = addFrame([ 19.17,  35.63, -0.49],  [0.0498143,-0.0133815,-0.998593,-0.0123221], robot,'p40','p40j','oframe');
    robot = addFrame([-21.65,  35.64, -0.22],  [0.0498108,-0.0133915,-0.998593,-0.0123259], robot,'p50','p50j','oframe');
    robot = addFrame([-21.65,  35.64, -0.22],  [0.0498136,-0.0133915,-0.998593,-0.0123257], robot,'p60','p60j','oframe');

    % Create figure and axes
    fig = figure('Name', 'Phase 2 - RAPID Simulation');
    ax  = axes('Parent', fig);

    show(robot, config, 'Parent', ax, 'Visuals', 'on', 'Frames', 'off');

    xlim(ax,[-0.5 1.2]); 
    ylim(ax,[-0.7 0.7]); 
    zlim(ax,[-0.1 1.5]);

    view(ax,[135 25]); 
    grid(ax,'on'); 
    camzoom(ax,0.7); 
    hold(ax,'on')

    % ── Mirrors the RAPID draw() procedure exactly ──
    %   MoveJ p10  → MoveL p20 → p30 → p40 → p50 → back to p20
    disp('Phase 2: Running RAPID path...');

    MoveL('p10', 'p20', robot, 't4', 30, ax);
    MoveL('p20', 'p30', robot, 't4', 30, ax);
    MoveL('p30', 'p40', robot, 't4', 30, ax);
    MoveL('p40', 'p50', robot, 't4', 30, ax);
    MoveL('p50', 'p20', robot, 't4', 30, ax);
    MoveL('p20', 'p10', robot, 't4', 30, ax);

    disp('Phase 2: Done!')

end


%% ===================== FUNCTIONS =====================

function Jconf = MoveL(startPose, endPose, robot, toolName, N, ax)
% MoveL — straight-line Cartesian motion with IK
%   startPose : config struct, 4x4 tform, or frame name string
%   endPose   : config struct, 4x4 tform, or frame name string
%   robot     : RigidBodyTree
%   toolName  : string name of the tool frame
%   N         : number of interpolation steps
%   ax        : axes handle to render into

    % ── Defaults ──
    if nargin < 4 || isempty(toolName)
        toolName = string(robot.BodyNames{end});
    else
        toolName = string(toolName);
    end

    if nargin < 5 || isempty(N)
        N = 30;
    end

    if nargin < 6 || isempty(ax)
        ax = gca;
    end

    robot.DataFormat = 'struct';

    % Starting IK seed
    if isConfigStruct(startPose)
        IKconf = startPose;
    else
        IKconf = homeConfiguration(robot);
    end
    Guess = IKconf;

    % Convert inputs to 4x4 transforms
    TFstart = toTform(startPose, robot, IKconf, toolName);
    TFend   = toTform(endPose,   robot, IKconf, toolName);

    % Extract position and rotation
    p0 = TFstart(1:3, 4);
    p1 = TFend(1:3,   4);
    q0 = rotm2quat(TFstart(1:3, 1:3));
    q1 = rotm2quat(TFend(1:3,   1:3));

    % IK solver setup
    ik      = inverseKinematics('RigidBodyTree', robot);
    weights = [1 1 1 1 1 1];   % [X Y Z Roll Pitch Yaw]

    % ── Interpolation loop ──
    for i = 1:N
        t = (i - 1) / (N - 1);

        % Linear position interpolation
        p = (1 - t) * p0 + t * p1;

        % Quaternion Nlerp (normalised linear interpolation for rotation)
        q = (1 - t) * q0 + t * q1;
        q = q / norm(q);

        % Build 4x4 target transform for this step
        T          = eye(4);
        T(1:3,1:3) = quat2rotm(q);
        T(1:3,4)   = p;

        % Solve IK and use previous solution as warm start
        [Solve, ~] = ik(toolName, T, weights, Guess);
        Guess      = Solve;

        % Visualise this step
        show(robot, Solve, ...
            'Parent',        ax,    ...
            'PreservePlot',  false, ...
            'Visuals',       'on',  ...
            'FastUpdate',    false);
        drawnow
    end

    Jconf = Guess;
end

% ── Helper: check if input is a joint config struct ──
function tf = isConfigStruct(x)
    tf = isstruct(x) && ~isempty(x) && ...
         isfield(x, 'JointName') && isfield(x, 'JointPosition');
end

% ── Helper: convert any valid input to a 4x4 transform ──
function T = toTform(x, robot, qSeed, toolName)
    if isnumeric(x) && isequal(size(x), [4 4])
        T = x;
        return;
    end
    if isConfigStruct(x)
        T = getTransform(robot, x, toolName);
        return;
    end
    if isstring(x) || ischar(x)
        T = getTransform(robot, qSeed, string(x));
        return;
    end
    error('MoveL:BadInput', ...
        'Input must be a 4x4 transform, a config struct, or a frame name string.');
end

% ── Helper: add a fixed frame to the robot tree ──
function robot = addFrame(Trans, q, robot, name, jointname, parentname)
    R     = quat2rotm(q);
    T     = [[R; [0 0 0]], [(Trans ./ 1000)'; 1]];
    frame = rigidBody(name);
    jnt   = rigidBodyJoint(jointname, 'fixed');
    setFixedTransform(jnt, T);
    frame.Joint = jnt;
    addBody(robot, frame, parentname);
end
