% The A start Grid (A*) algorithm
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
x=int16(x) % make it integer as planner except only integers
y=int16(y)
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

%% Create A*Grid Planner
tic
%[x,y] = ginput(2)
start = double([abs(y(1)-1665) abs(x(1))])
goal= double([abs(y(2)-1665) abs(x(2))])
planner = plannerAStarGrid(map);
[path,debugInfo]=plan(planner,start,goal);
toc

%%  visualize the path  
figure(5)
show(planner)
path_lenght=debugInfo.PathCost
MEMORY_used = monitor_memory_whos( )
