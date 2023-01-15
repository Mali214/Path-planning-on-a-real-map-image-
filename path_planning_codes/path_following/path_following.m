% The probabilistic roadmap (PRM) algorithm
img=imread('msa_17b.JPG'); %read MSA MAP image  

I1=rgb2gray(img); % changing the map from rgb to gray image 
conn=8;

b = imsharpen(I1,'Radius',3,'Amount',4); %sharpens the MSA MAP image 
C1=imadjust(b,[],[],1.7);                   % add contrast to the image  with gamma = 1.7
figure(9)
img1=imresize(flipud(img),1);     
imshow(img1); title(' orginal MSA map  ');axis on
set(gca,'YDir','normal')
list = {'Station1','Station2','Station3',...                   
'Station4'};
[indx,tf] = listdlg('PromptString',{'please the number of station that you want to go to .',''},...
    'SelectionMode','single','ListString',list);

x=[1.722559497816594e+03;1.520669759825328e+03];
y=[1.443760917030566e+02;1.015900109170306e+03];
x1=[1.536661026200874e+03;6.551424672489084e+02];
y1=[1.035889192139738e+03;1.365709061135371e+03];
x2=[6.551424672489084e+02;2.473651746724891e+02];
y2=[1.367707969432315e+03;2.982920305676855e+02];
x3=[2.413684497816595e+02;1.702570414847162e+03];
y3=[1.343815502183404e+02;1.503728165938862e+02];

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


yi=~BW1;

figure(4)
imshow(yi);
imageNorm = double(~yi)/255;
imageOccupancy = imageNorm;       

map=binaryOccupancyMap(imageOccupancy,1);             %create occupancy map 

%% Define Vehicle and Pure Pursuit controller
robot = differentialDriveKinematics("TrackWidth", 20, "VehicleInputs", "VehicleSpeedHeadingRate");
controller = controllerPurePursuit;
controller.DesiredLinearVelocity = 20;
controller.MaxAngularVelocity = 15;
controller.LookaheadDistance = 1.9;
controller1 = controllerPurePursuit;
controller1.DesiredLinearVelocity = 20;
controller1.MaxAngularVelocity = 15;
controller1.LookaheadDistance = 1.9;
controller2 = controllerPurePursuit;
controller2.DesiredLinearVelocity = 20;
controller2.MaxAngularVelocity = 15;
controller2.LookaheadDistance = 1.9;
controller3 = controllerPurePursuit;
controller3.DesiredLinearVelocity = 20;
controller3.MaxAngularVelocity = 15;
controller3.LookaheadDistance = 1.9;
robotRadius = 3;
%% Map parameters
mapInflated = copy(map);
inflate(mapInflated,robot.TrackWidth/4);
show(mapInflated)

startLocation = [x(1) y(1) ];            % Start pose [x y ]
endLocation = [x(2) y(2) ];       % Goal pose [x y ]
startLocation1 = [x1(1) y1(1) ];            % Start pose [x y ]
endLocation1 = [x1(2) y1(2) ];
startLocation2 = [x2(1) y2(1) ];            % Start pose [x y ]
endLocation2 = [x2(2) y2(2) ];
startLocation3 = [x3(1) y3(1) ];            % Start pose [x y ]
endLocation3 = [x3(2) y3(2) ];

%% Create PRM Planner
tic
prm = mobileRobotPRM;   
prm.Map = mapInflated;
prm.NumNodes = 3000; %Create a simple roadmap with 3000 nodes.
prm.ConnectionDistance = 150; % Distance between nodes = 150

%% Plan a path and set controller waypoints 
path = findpath(prm, startLocation, endLocation);
path1 = findpath(prm, startLocation1, endLocation1);
path2 = findpath(prm, startLocation2, endLocation2);
path3 = findpath(prm, startLocation3, endLocation3);

show(prm)
zc = zeros(size(path,1),1);
path_o = [ path, zc];
dubinsSpace = stateSpaceDubins([0 1831;0 1665;0 0])
pathobj = navPath(dubinsSpace)
append(pathobj, path_o);
len = pathLength(pathobj)
toc

controller.Waypoints = path;
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
controller1.Waypoints = path1;
robotInitialLocation1 = path1(1,:);
robotGoal1 = path1(end,:);
initialOrientation = 0;

controller2.Waypoints = path2;
robotInitialLocation2 = path2(1,:);
robotGoal2 = path2(end,:);

controller3.Waypoints = path3;
robotInitialLocation3 = path3(1,:);
robotGoal3 = path3(end,:);

robotCurrentPose = [robotInitialLocation initialOrientation]';
initialOrientation1 = 0
robotCurrentPose1 = [robotInitialLocation1 initialOrientation1]';
robotCurrentPose2 = [robotInitialLocation2 initialOrientation1]';
robotCurrentPose3 = [robotInitialLocation3 initialOrientation1]';
distanceToGoal = norm(robotInitialLocation - robotGoal);
distanceToGoal1 = norm(robotInitialLocation1 - robotGoal1);
distanceToGoal2 = norm(robotInitialLocation2 - robotGoal2);
distanceToGoal3 = norm(robotInitialLocation3 - robotGoal3);
goalRadius = 30;
%% visualize path



%% Path following simulation loop
if indx==1
    Stations(distanceToGoal,goalRadius,path,robotCurrentPose,robot,controller,robotGoal,img1)
    f = msgbox('you have arrivied to Station 1');
elseif indx==2
    Stations(distanceToGoal,goalRadius,path,robotCurrentPose,robot,controller,robotGoal,img1)
    f = msgbox('you have arrivied to Station 1');
    Stations(distanceToGoal,goalRadius,path1,robotCurrentPose1,robot,controller1,robotGoal1,img1)
    f = msgbox('you have arrivied to Station 2');
elseif indx==3
Stations(distanceToGoal,goalRadius,path,robotCurrentPose,robot,controller,robotGoal,img1)
f = msgbox('you have arrivied to Station 1');
Stations(distanceToGoal,goalRadius,path1,robotCurrentPose1,robot,controller1,robotGoal1,img1)
f = msgbox('you have arrivied to Station 2');
Stations(distanceToGoal2,goalRadius,path2,robotCurrentPose2,robot,controller2,robotGoal2,img1)
f = msgbox('you have arrivied to Station 3');
elseif indx==4       
Stations(distanceToGoal,goalRadius,path,robotCurrentPose,robot,controller,robotGoal,img1)
f = msgbox('you have arrivied to Station 1');
Stations(distanceToGoal,goalRadius,path1,robotCurrentPose1,robot,controller1,robotGoal1,img1)
f = msgbox('you have arrivied to Station 2');
Stations(distanceToGoal2,goalRadius,path2,robotCurrentPose2,robot,controller2,robotGoal2,img1)
f = msgbox('you have arrivied to Station 3');
Stations(distanceToGoal3,goalRadius,path3,robotCurrentPose3,robot,controller3,robotGoal3,img1)
f = msgbox('you have arrivied to Station 4');
end






function Stations(distanceToGoal,goalRadius,path,robotCurrentPose,robot,controller,robotGoal,img1)
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);
reset(vizRate);
frameSize = robot.TrackWidth/0.8;
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    imshow(img1); title(' orginal MSA map  ');axis on
    set(gca,'YDir','normal')
    hold all

   % Plot path each instance so that it stays persistent while robot mesh
   % moves
    plot(path(:,1), path(:,2),"k--d")
    
   % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot=axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    ylim([0 1665])
    xlim([0 1831])
    
    waitfor(vizRate);
   
        
end
end
