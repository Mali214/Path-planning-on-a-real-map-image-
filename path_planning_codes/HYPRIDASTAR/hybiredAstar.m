tic
img=imread('msa_17b.JPG');

I1=rgb2gray(img);
conn=8;

b = imsharpen(I1,'Radius',3,'Amount',4);
C1=imadjust(b,[],[],1.7);
figure(9)
img1=imresize(flipud(img),1);
imshow(img1); title(' orginal MSA map  ');axis on
set(gca,'YDir','normal')
[x,y] = ginput(2)

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
Fss=medfilt2(bw,[21 21]) ;
BW = imfill(Fss,4,'holes');
Fs1=medfilt2(bw,[9 9]) ;
BW1= imfill(Fs1,4,'holes');


yi=~BW1;

figure(4)
imshow(yi);
imageNorm = double(~yi)/255;
imageOccupancy = imageNorm;

map=binaryOccupancyMap(imageOccupancy,1);
validator = validatorOccupancyMap;
validator.Map = map;
show(map)
%[x,y] = ginput(2)




%[x,y] = ginput(2)
startLocation = [x(1) y(1) pi/2];
endLocation = [x(2) y(2) -pi/2];
planner = plannerHybridAStar(validator);
path = plan(planner,startLocation,endLocation);
figure(5)

show(planner)

len = pathLength(path)
toc
 %Initialize the figure
 

