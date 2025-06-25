%% Motion Planning with Robust Start/Goal Handling
clc; clear; close all;

%% 1. Load and Prepare Your Custom Map
mapImage = 'C:\Users\MEET\Pictures\Screenshots\map.png';
mapData = imread(mapImage);

% Convert to binary occupancy map (black=obstacle, white=free)
if size(mapData,3) == 3
    mapData = rgb2gray(mapData);
end
binaryMap = ~imbinarize(mapData); % Invert so black=free, white=obstacle
map = occupancyMap(binaryMap, 1); % 1 cell/meter resolution

%% 2. Display Map and Get Dimensions
figure;
show(map);
title('3BHK House Map for Path Planning');
hold on;
gridSize = size(binaryMap);
disp(['Map dimensions: ', num2str(gridSize(1)), 'x', num2str(gridSize(2))]);

%% 3. Find Valid Start and Goal Positions
% Get free space indices
[freeRows, freeCols] = find(binaryMap == 0);
freePositions = [freeCols, freeRows]; % Convert to [x,y] format

% Ensure we have enough free space
if numel(freeRows) < 2
    error('Not enough free space in the map for path planning');
end

% Select random start and goal positions with minimum separation
minSeparation = min(gridSize)/3;
maxAttempts = 100;

for attempt = 1:maxAttempts
    % Randomly select indices
    randIndices = randperm(length(freeRows), 2);
    startCandidate = freePositions(randIndices(1),:);
    goalCandidate = freePositions(randIndices(2),:);
    
    % Check separation
    if norm(startCandidate - goalCandidate) >= minSeparation
        start = [startCandidate(1), startCandidate(2), 0]; % [x, y, theta]
        goal = [goalCandidate(1), goalCandidate(2), 0];
        break;
    end
end

% Plot positions
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(start(1), start(2), 'Start', 'Color', 'g', 'FontWeight', 'bold');
text(goal(1), goal(2), 'Goal', 'Color', 'r', 'FontWeight', 'bold');

%% 4. Configure Path Planner with Improved Parameters
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss, 'Map', map);
sv.ValidationDistance = 0.5; % Increased for better performance
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

planner = plannerRRT(ss, sv, ...
    'MaxIterations', 10000, ... % Increased iterations
    'MaxConnectionDistance', 10, ... % Increased connection distance
    'GoalBias', 0.1); % Slight bias toward goal

%% 5. Plan Path with Validation
rng(42); % For reproducibility
[pthObj, solnInfo] = plan(planner, start, goal);

% Check if path was found
if isempty(pthObj.States)
    error('Path planner failed to find a valid path between start and goal');
end

% Plot raw path
plot(pthObj.States(:,1), pthObj.States(:,2), 'b-', 'LineWidth', 1);

%% 6. Optimize Path with Validation
try
    optOptions = optimizePathOptions;
    optOptions.ObstacleSafetyMargin = 1.5;
    optOptions.MaxIterations = 200;
    
    optPath = optimizePath(pthObj.States, map, optOptions);
    
    % Plot optimized path
    plot(optPath(:,1), optPath(:,2), 'm-', 'LineWidth', 2);
    
    % Calculate metrics
    metrics = pathmetrics(pthObj, sv);
    disp(['Path Length: ', num2str(metrics.Length), ' meters']);
    disp(['Min Clearance: ', num2str(min(metrics.Clearance)), ' meters']);
catch ME
    warning('Path optimization failed, using raw path');
    disp(ME.message);
    optPath = pthObj.States;
end

%% 7. Final Visualization
legend('Start', 'Goal', 'RRT Path', 'Optimized Path');
hold off;