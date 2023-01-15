% %
% Artificial  potential filed (APF) algorithm
clear;
close all;
img=imread('msa_17b.JPG'); %read MSA MAP image 

I1=rgb2gray(img);  % changing the map from rgb to gray image
conn=8;

b = imsharpen(I1,'Radius',3,'Amount',4);   %sharpens the MSA MAP image 
C1=imadjust(b,[],[],1.7);                 % add contrast to the image  with gamma = 1.7
figure(9)
imshow(img); title(' orginal MSA map  ');
disp('Please select the start point then the goal point on the MAP' )
[xs,ys] = ginput(2) % let user select the start point then the goal point

figure(1)
subplot(1,2,1);imshow(img); title(' orginal MSA map  ');
subplot(1,2,2);imshow(I1); title('Gray image of  MSA map ');
figure(2)
subplot(1,2,1);imshow(b); title(' MSA map after addinf sharpen filter  ');
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
Fss=medfilt2(bw,[21 21]) ;  % Apply Median filtter to the image with mask size of 21 *21  
BW = imfill(Fss,4,'holes');
Fs1=medfilt2(bw,[9 9]) ;  % Apply Median filtter to the image with mask size of 9*9  
BW1= imfill(Fs1,4,'holes');
yi=~BW1; 
for i=1510:1:1545
for j=1210:1:1249

     yi(i,j)=1;
end
  
end 
for i=1542:1:1549
for j=1240:1:1290

     yi(i,j)=0;
end
  
end 
figure, imshow(yi);title('MSA Free road map');
%% 
goal = [xs(2) ys(2)];   % Start pose [x y ]
start = [xs(1) ys(1)];    % GOAL pose [x y ]
nrows = size(yi,1);
ncols = size(yi,2);
[x, y] = meshgrid (1:ncols, 1:nrows);


%% Compute distance transform
tic
d = bwdist(yi);

% Rescale and transform distances

d2 = (d/100) + 1;

d0 = 100;
nu = 500;

repulsive = nu*((1./d2 - 1/d0).^2);

repulsive (d2 > d0) = 0;


%% Display repulsive potential

figure;
m = mesh (repulsive);
m.FaceLighting = 'phong';
axis equal;

title ('Repulsive Potential');

%% Compute attractive force
xi = 1/500;
attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );
figure;
m = mesh (attractive);
m.FaceLighting = 'phong';
axis equal;
title ('Attractive Potential');

%% Display 2D configuration space
figure;
imshow(yi);
hold on;
plot (goal(1), goal(2), 'r.', 'MarkerSize', 50);
plot (start(1), start(2), 'g.', 'MarkerSize', 50);
hold off;
legend('Goal', 'Start')
title ('Configuration Space');

%% Combine terms
f = attractive + repulsive;
figure;
m = mesh (f);
m.FaceLighting = 'phong';
axis equal;
title ('Total Potential');

%% Plan route
route = GradientBasedPlanner (f, start, goal, 1000);
toc
nr = size(route,1);
nc = size(route,2);
route1=route(1:20:nr,1);
route2=route(1:20:nr,2);
zc = zeros(size(route1,1),1);
path_o = double([route1,route2, zc]);
dubinsSpace = stateSpaceDubins([0 1665;0 1831;0 0]);
pathobj = navPath(dubinsSpace);
append(pathobj, path_o);
len = pathLength(pathobj)
MEMORY_used = monitor_memory_whos( )
%% Plot the energy surface
figure;
m = mesh (f);
axis equal;
%% Plot ball sliding down hill
animate = 1;
if animate
    [sx, sy, sz] = sphere(10);
    scale =10 ;
    sx = scale*sx;
    sy = scale*sy;
    sz = scale*(sz+1);
    hold on;
    p = mesh(sx, sy, sz);
    p.FaceColor = 'red';
    p.EdgeColor = 'none';
    p.FaceLighting = 'phong';
    hold off;
for i = 1:size(route,1)
        P = round(route(i,:));
        z = f(P(2), P(1));

        p.XData = sx + P(1);
        p.YData = sy + P(2);
        p.ZData = sz + f(P(2), P(1));
        drawnow;
        drawnow;

    end
end

%% quiver plot
[gx, gy] = gradient (-f);
skip = 10;
figure;
xidx = 1:skip:ncols;
yidx = 1:skip:nrows;
quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.5);
axis ([1 ncols 1 nrows]);
hold on;
ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
xlabel('X')
ylabel('Y')

