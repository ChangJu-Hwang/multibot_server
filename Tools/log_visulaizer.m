clc; clear; close all;

addpath("yamlParser/");

%% User Setting
% mapImageFileName = 'inno_1st_floor';
% mapResolution = 0.1;
% mapImageFileName = '12x12_grid_map';
MapFileName = 'testbed';

yamlLogFileName = 'CPBS_Log';
yamlAgentConfigFileName = 'agentConfig';
yamlTaskFileName = 'multibot_task';

timeResolution = 0.01;
mapResolution = 0.1;

% visualizationMargin_x = 0.5;
% visualizationMargin_y = 0.5;
visualizationMargin_x = 4;
visualizationMargin_y = 4;
% titleSpec.('Title') = 'Narrow Corridor(Resolution: 0.1m)';
titleSpec.('Title') = "testbed(Resolution: ";
titleSpec.('FontSize') = 17;
nodeSpec.('Size') = 10;
pathSpec.('Width') = 5;
pathSpec.('Alpha') = 0.7;
agentSpec.('Ratio') = 0.7;
agentSpec.('BoundaryWidth') = 2;
agentSpec.('InnerWidth') = 2;

%% Import Agent Data
% agents = yaml.loadFile("../Instances/agents/"+yamlAgentConfigFileName+".yaml");
agentConfigs = yaml.loadFile("../Instances/agents/"+yamlAgentConfigFileName+".yaml");
agentConfigs = agentConfigs.x___.ros__parameters;

%% Import Log Data
log = yaml.loadFile("../Results/"+yamlLogFileName+".yaml");

agents = log.Log;
agentConfigNames = fields(agentConfigs);
%% Merge Agent and Log Data
for logIter=1:numel(log.Log)
    logData = log.Log{logIter};

    for agentConfigIter=1:numel(agentConfigNames)
        if not(strcmp(agentConfigNames{agentConfigIter},logData.type))
            continue;
        end

        agentConfig = agentConfigs.(agentConfigNames{agentConfigIter});

        agents{logIter}.('size') = agentConfig.('size');
        agents{logIter}.('linear_velocity') = agentConfig.('linear').('velocity');
        agents{logIter}.('linear_acceleration') = agentConfig.('linear').('acceleration');
        agents{logIter}.('angular_velocity') = agentConfig.('angular').('velocity');
        agents{logIter}.('angular_acceleration') = agentConfig.('angular').('acceleration');
    end
end
clear agentConfigIter logIter j log logData;

%% Customize some information for visualization
for agentIter=1:numel(agents)
    agents{agentIter}.('color') = rand(1,3);
end
clear agentIter;

%% Generate Trajectory for each agent
makeSpan = 0;
xScope = [Inf, -Inf];
yScope = [Inf, -Inf];
for agentIter=1:numel(agents)
    makeSpan = max(makeSpan, agents{agentIter}.cost);
    path = agents{agentIter}.path;
    for pathIter = 1:numel(path)
        local_start = cell2mat(path{pathIter}.Start);
        local_goal  = cell2mat(path{pathIter}.Goal);

        local_min = min(local_start, local_goal);
        local_max = max(local_start, local_goal);

        xScope(1) = min(xScope(1), local_min(1));
        xScope(2) = max(xScope(2), local_max(1));
        yScope(1) = min(yScope(1), local_min(2));
        yScope(2) = max(yScope(2), local_max(2));

        clear local_start local_goal local_min local_max
    end
    clear pathIter path;
end
clear agentIter;
makeSpan = round(makeSpan / timeResolution) * timeResolution;
xScope = xScope + abs((xScope(2)-xScope(1))) * visualizationMargin_x * [-1,1];
yScope = yScope + abs((yScope(2)-yScope(1))) * visualizationMargin_y * [-1,1];

for agentIter=1:numel(agents)
    agents{agentIter}.('trajectory') = generateTrajectory(agents{agentIter}, timeResolution, makeSpan);
end
clear agentIter;

%% Visualization
% mapImageFileDir = "../Instances/map/"+ mapImageFileName + ".png";    clear mapImageFileName;
% mapImage = imread(mapImageFileDir); clear mapImageFileDir;
% grayMapImage = rgb2gray(mapImage);  clear mapImage;
% 
% bwMapImage = grayMapImage < 205 + 0.5;  clear grayMapImage;
% worldMap = binaryOccupancyMap(bwMapImage, 1/mapResolution);
% clear bwMapImage;
image = imread("../Instances/maps/"+MapFileName+"/"+MapFileName+".png");
image = imresize(image,1);
maskG = image(:,:,1) < 40 & image(:,:,2) > 70 & image(:,:,3) < 40;

newImage = image;
selectedmask = repmat(maskG,[1,1,3]);
newImage(selectedmask) = 0;
newImage = newImage + uint8(selectedmask.*permute([0,0,0],[1,3,2]));

grayimage = rgb2gray(newImage);
bwimage = grayimage < 205 + 0.5;

titleSpec.('Title') = titleSpec.('Title') + num2str(mapResolution) + "m)";
worldMap = binaryOccupancyMap(bwimage,1/mapResolution);
clear rowIter row mapData;

xScope(1) = max(xScope(1), worldMap.XWorldLimits(1));
xScope(2) = min(xScope(2), worldMap.XWorldLimits(2));
yScope(1) = max(yScope(1), worldMap.YWorldLimits(1));
yScope(2) = min(yScope(2), worldMap.YWorldLimits(2));

figure('units','normalized', ...
       'OuterPosition',[0.1 0.1 0.9 0.9], ...
       'Name','Log Visualizer', ...
       'NumberTitle','off');
% for agentIter = 1:numel(agents)
%     traj = agents{agentIter}.trajectory;
%     t = traj(:,1);
%     subplot(numel(agents)*3,3,(agentIter-1)*3+1);
%         x = traj(:,2);
%         plot(t,x);
%         title(agents{agentIter}.name+': x');
%     subplot(numel(agents)*3,3,(agentIter-1)*3+2);
%         y = traj(:,3);
%         plot(t,y);
%         title(agents{agentIter}.name+': y');
%     subplot(numel(agents)*3,3,(agentIter-1)*3+3);
%         theta = traj(:,4);
%         plot(t,theta);
%         title(agents{agentIter}.name+': theta');
% end

collisionLog = {};
for t = 0:timeResolution:makeSpan
    for firstAgentIter = 1:numel(agents)-1
        firstPose = agents{firstAgentIter}.trajectory(uint8(t/timeResolution+1),2:3);

        for secondAgentIter = firstAgentIter + 1:numel(agents)
            secondPose = agents{secondAgentIter}.trajectory(uint8(t/timeResolution+1),2:3);

            if (norm(firstPose - secondPose) < agents{firstAgentIter}.size + agents{secondAgentIter}.size)
                collisionLog{end+1,1} = t;
                collisionLog{end,2} = agents{firstAgentIter}.name;
                collisionLog{end,3} = agents{secondAgentIter}.name;
                collisionLog{end,4} = firstPose;
                collisionLog{end,5} = secondPose;
            end
        end
    end
end

if (numel(collisionLog) ~= 0)
    fprintf('Collision Occurs');
end

for t = 0:timeResolution:makeSpan
    visualizeMap(worldMap, titleSpec, xScope, yScope);
    visualizePath(agents, mapResolution, pathSpec);
    visualizeNode(agents, mapResolution, nodeSpec);
    visualizeTraj(agents, t, mapResolution, agentSpec);

    pause(0.005);
    hold off;
end
clear makeSpan xScope yScope t;

%% Function
% Trajectory Generation
function traj = generateTrajectory(agent_, timeResolution_, makeSpan_)
    path = agent_.path;
    traj = zeros(0,4);
    traj(end+1,:) = [0.0, cell2mat(path{1}.Start)];

    v = agent_.linear_velocity;     a = agent_.linear_acceleration;
    w = agent_.angular_velocity;    alpha = agent_.angular_acceleration;
    
    for pathIter = 1:numel(path)
        local_start = cell2mat(path{pathIter}.Start);
        local_goal  = cell2mat(path{pathIter}.Goal);
        local_timeInterval = cell2mat(path{pathIter}.TimeInterval);

        while(abs(local_timeInterval(1) - traj(end,1)) >= timeResolution_)
            traj(end+1,:) = traj(end,:);
            traj(end,1) = traj(end,1) + timeResolution_;
        end

        diff = local_goal - local_start;
        angle = diff(3);
        while(abs(angle)>pi)
            angle = angle - 2*pi * angle / norm(angle);
        end

        rotationTime = durationComputer(angle, w, alpha);
        
        t = timeResolution_;
        startTime = traj(end,1);
        while(t < local_timeInterval(2)-local_timeInterval(1))
            if(t < rotationTime)
                traj(end+1,:) = [startTime + t, local_start];
                traj(end,4) = displacementComputer(local_start(3), local_start(3) + angle, t, w, alpha);
            else
                traj(end+1,:) = [startTime + t, local_goal];

                v_x = v * cos(local_goal(3)); v_y = v * sin(local_goal(3));
                a_x = a * cos(local_goal(3)); a_y = a * sin(local_goal(3));
                transitionTime = t - rotationTime;

                traj(end,2) = displacementComputer(local_start(1), local_goal(1), transitionTime, v_x, a_x);
                traj(end,3) = displacementComputer(local_start(2), local_goal(2), transitionTime, v_y, a_y);
            end
            t = t + timeResolution_;
        end
    end

    while(abs(makeSpan_ - traj(end,1)) >= timeResolution_)
        traj(end+1,:) = traj(end,:);
        traj(end,1) = traj(end,1) + timeResolution_;
    end
end

function duration = durationComputer(s_, v_, a_)
    if(abs(s_) > v_ * v_ / a_)
        duration = abs(s_) / v_ + v_ / a_;
    else
        duration = sqrt(4 * abs(s_) / a_);
    end
end

function pos = displacementComputer(start_, goal_, t_, v_, a_)
    max_s_ =  abs(goal_ - start_);

    if(max_s_ > 1e-8)
        sign = 1;
        if(goal_ - start_ < 0)
            sign = -1;
        end
    
        v_ = abs(v_);   a_ = abs(a_);
        if(max_s_ > v_*v_ / a_)
            if(t_ < v_/a_)
                s = abs(0.5*a_*t_*t_);
            elseif(t_ < max_s_ / v_)
                s = abs(v_*t_ - 0.5*v_*v_/a_);
            else
                s = abs(max_s_ - 0.5 * a_ * (max_s_ / v_ + v_ / a_ - t_)^2);
            end
        else
            if(t_ < sqrt(max_s_ / a_))
                s = abs(0.5*a_*t_*t_);
            else
                s = abs(max_s_ - 0.5*a_*(2*sqrt(max_s_/a_)-t_)^2);
            end
        end
    
        pos = start_ + s * sign;
    else
        pos = start_;
    end

    
end

% Visualization
function visualizeMap(map_, titleSpec_, xScope_, yScope_)
    show(map_);
    title(titleSpec_.Title,'FontSize',titleSpec_.FontSize);
    grid on;    grid minor; hold on;

    xlim(xScope_);  ylim(yScope_);
    set(gca, 'xtick', xScope_(1):0.5:xScope_(2));
    set(gca, 'ytick', yScope_(1):0.5:yScope_(2));
end

function visualizePath(agents_, mapResolution_, pathSpec_)
    for agentIter = 1:numel(agents_)
        path = agents_{agentIter}.path;

        pathSpec_.('Color') = agents_{agentIter}.color;
        for pathIter = 1:numel(path)
            local_start = cell2mat(path{pathIter}.Start) + mapResolution_/2;
            local_goal  = cell2mat(path{pathIter}.Goal)  + mapResolution_/2;

            x = linspace(local_start(1),local_goal(1));
            y = linspace(local_start(2),local_goal(2));
            plot(x,y,':',...
                'LineWidth',pathSpec_.Width,...
                'Color',[pathSpec_.Color,pathSpec_.Alpha]);
        end
    end
end

function visualizeNode(agents_, mapResolution_, nodeSpec_)
    for agentIter = 1:numel(agents_)
        path = agents_{agentIter}.path;

        nodeSpec_.('FaceColor') = agents_{agentIter}.color;
        plotMarker(path{1}.Start, mapResolution_, nodeSpec_);
        plotMarker(path{numel(path)}.Goal, mapResolution_, nodeSpec_);
%         for pathIter = 1:numel(path)
%             plotMarker(path{pathIter}.Goal, mapResolution_, nodeSpec_);
%         end
    end
end

function visualizeTraj(agents_, t_, mapResolution_, agentSpec_)
    for agentIter = 1:numel(agents_)
        agentSpec_.('Color') = agents_{agentIter}.color;
        agentSpec_.('Size')  = agents_{agentIter}.size;

        index = find(agents_{agentIter}.trajectory(:,1)>=t_-1e-8,1);
        state = agents_{agentIter}.trajectory(index, 2:4);
        
        plotCirleAgent(state, mapResolution_, agentSpec_);
    end
end

function plotMarker(pos_, mapResolution_, nodeSpec_)
    x = cell2mat(pos_(1)) + mapResolution_ / 2;
    y = cell2mat(pos_(2)) + mapResolution_ / 2;

    plot(x, y,...
        'Marker','o',...
        'MarkerEdgeColor','none',...
        'MarkerFaceColor',nodeSpec_.FaceColor,...
        'MarkerSize',nodeSpec_.Size);
end

function plotCirleAgent(state_, mapResolution_, agentSpec_)
    i = state_(1) + mapResolution_ / 2; j = state_(2) + mapResolution_ / 2;

    radius = agentSpec_.Size * agentSpec_.Ratio;

    rectangle('Position',[i-radius, j-radius, 2*radius, 2*radius],...
              'Curvature',[1,1],...
              'FaceColor',agentSpec_.Color,...
              'LineWidth',agentSpec_.BoundaryWidth,...
              'EdgeColor','none');
    
    x = linspace(i,i+radius*cos(state_(3)));
    y = linspace(j,j+radius*sin(state_(3)));
    plot(x,y, 'LineWidth',agentSpec_.InnerWidth,'Color',1-agentSpec_.Color);    hold on;

    x = linspace(i-radius*cos(state_(3)+pi/2),i+radius*cos(state_(3)+pi/2));
    y = linspace(j-radius*sin(state_(3)+pi/2),j+radius*sin(state_(3)+pi/2));
    plot(x,y, 'LineWidth',agentSpec_.InnerWidth,'Color',1-agentSpec_.Color);    hold on;
end