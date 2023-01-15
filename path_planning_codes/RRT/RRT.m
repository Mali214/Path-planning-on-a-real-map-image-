% the Rapidly-exploring Random Tree (RRT) algorithm
clear all 
img=imread('msa_17b.JPG'); %read MSA MAP image  

I1=rgb2gray(img); % changing the map from rgb to gray image 
conn=8;

b = imsharpen(I1,'Radius',3,'Amount',4); %sharpens the MSA MAP image 
C1=imadjust(b,[],[],1.7);                   % add contrast to the image  with gamma = 1.7
figure(9)
img1=imresize(flipud(img),1);     
imshow(img1); title(' orginal MSA map  ');axis on
set(gca,'YDir','normal')
disp('Please select the start point then the goal point on the MAP' )
[x,y] = ginput(2) % let user select the start point then the goal point

figure(1)
subplot(1,2,1);imshow(img); title(' orginal MSA map  ');

subplot(1,2,2);imshow(I1); title('Gray image of  MSA map ');
figure(2)
subplot(1,2,1);imshow(b); title(' MSA map after addind sharpen filter  ');
subplot(1,2,2);imshow(C1); title('MSA map after adding sharpen  and contrast filters ');
bw=C1;

for i=1:1:size(bw,1)
for j=1:1:size(bw,2)
if bw(i,j)>100 && bw(i,j)<200
     bw(i,j)=0;

    
else 
   bw(i,j)=1;
end
  
end 
end
Fss=medfilt2(bw,[21 21]);     % Apply Median filtter to the image with mask size of 21 *21  
BW = imfill(Fss,4,'holes');    
Fs1=medfilt2(bw,[9 9]) ;      % Apply Median filtter to the image with mask size of 9*9
BW1= imfill(Fs1,4,'holes');   


yi=~BW1

figure(4)
imshow(yi);
imageNorm = double(~yi)/255;
imageOccupancy = imageNorm;       

map=binaryOccupancyMap(imageOccupancy,1);             %create occupancy map 

%% Define Vehicle
wheelRadius = 0.5;     % Wheel radius [m]
frontLen = 2;        % Distance from CG to front wheels [m]
rearLen = 2;         % Distance from CG to rear wheels [m]
vehicle = FourWheelSteering(wheelRadius,[frontLen rearLen]);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:10000;        % Time array

startPose = [x(1) y(1) 0];            % Start pose [x y theta]
goalPose = [x(2) y(2) -pi/2];       % Goal pose [x y theta]

% Create visualizer
viz = Visualizer2D;
viz.robotRadius = frontLen;
viz.hasWaypoints = false;
viz.mapName = 'map' 

%% Create RRT Planner
tic
inflate(map,0.25); % Inflate the map for planning

% State space
ss = stateSpaceDubins;
ss.MinTurningRadius = 4;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% State validator
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 5;

% Path planner
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 70;

%% Plan a path and visualize it
[plannedPath,solInfo] = plan(planner,startPose,goalPose);
if plannedPath.NumStates < 1
    disp('No path found. Please rerun the example');
end
interpolate(plannedPath,round(2*plannedPath.pathLength)); % Interpolate to approx. 2 waypoints per meter
viz(startPose)
visualizeRRTPath(plannedPath,solInfo,ss); % Helper function to visualize
len = pathLength(plannedPath)
toc
%% Define Pure Pursuit controller for path following
waypoints = plannedPath.States(:,1:2);
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 2;
controller.DesiredLinearVelocity = 3;
controller.MaxAngularVelocity = 3;

%% Path following simulation loop
r = rateControl(5/sampleTime); % Run at 5x speed
pose = zeros(3,numel(tVec));
pose(:,1) = startPose;
idx = 2;
dist = Inf;

while (idx < numel(tVec)) && (dist > 0.35)
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wheelSpeeds,steerAngles] = inverseKinematicsFrontSteer(vehicle,vRef,wRef);
    wheelSpeeds = wheelSpeeds([1 1]); % Use front wheel speed for both
    
    % Compute the velocities
    velB = forwardKinematics(vehicle,wheelSpeeds,steerAngles);
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
    
    % Update visualization
    viz(pose(:,idx))
    
    % Calculate distance to goal and update loop
    dist = norm( pose(1:2,idx) - goalPose(1:2)' );
    idx = idx+1;
    waitfor(r);
end

%% Helper function to visualize path
function visualizeRRTPath(plannedPath,solInfo,ss)
hold on
% Plot the path from start to goal
plot(plannedPath.States(:,1),plannedPath.States(:,2),'r--','LineWidth',1.5);
% Interpolate each path segment to be smoother and plot it
tData = solInfo.TreeData;
for idx = 3:3:size(tData,1)-2
    p = navPath(ss,tData(idx:idx+1,:));
    interpolate(p,10);
    plot(p.States(:,1),p.States(:,2),':','Color',[0 0.6 0.9]);
end
hold off
end