% The probabilistic roadmap (PRM) algorithm
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


%% Map parameters
mapInflated = copy(map);
show(mapInflated)

startLocation = [x(1) y(1) ];            % Start pose [x y ]
endLocation = [x(2) y(2) ];       % Goal pose [x y ]



%% Create PRM Planner
tic
prm = mobileRobotPRM;   
prm.Map = mapInflated;
prm.NumNodes = 3000; %Create a simple roadmap with 3000 nodes.
prm.ConnectionDistance = 150; % Distance between nodes = 150

%% Plan a path and set controller waypoints 
path = findpath(prm, startLocation, endLocation);
show(prm)
zc = zeros(size(path,1),1);
path_o = [ path, zc];
dubinsSpace = stateSpaceDubins([0 1831;0 1665;0 0])
pathobj = navPath(dubinsSpace)
append(pathobj, path_o);
len = pathLength(pathobj)
toc

 
monitor_memory_whos( )