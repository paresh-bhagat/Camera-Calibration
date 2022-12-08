%% Input checkboard image
clc;
clear all;

path = 'checkb8.jpg';
 

%% Getting input 2D coordinates from the image and 3D coordinates manually in homogenous coordinates.
 
img = imread(path);
image(img);
axis image

% 3D points measured manually
points_3D = [0 0 0 0 16 16 0 0 0 4 8 6 0 0 10 12;
             0 16 16 0 0 0 8 4 6 0 0 0 10 12 0 0;
             0 0 16 16 16 0 8 6 10 6 8 10 4 12 4 12;
             1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];  

[~,n] = size(points_3D);

% 2D points
points_2D = [];   

zoom on;
zi = 1;

for i=1:n
    [xi, yi, zi] = ginput(1);
    hold on;
    plot(xi, yi, 'ro')
    hold off;
    temp1 = [xi; yi; 1];
    points_2D =  [points_2D temp1];        
end

%% Normalize the 2D and 3D Data points 

%For 2D

%T matrices using centroid
T_temp = [1 0 -mean(points_2D(1,:));
          0 1 -mean(points_2D(2,:)); 
          0 0 1 ] ;

%find lamda for 2D points
total_sum = 0 ;
xy_center = T_temp*points_2D ;
for i = 1:6
    total_sum = total_sum + norm(xy_center(1:2,i)) ;
end
lamda_2D = 6*sqrt(2)/total_sum  ;

%Transformation matrix T
T = lamda_2D * T_temp ;
T(3,3) = 1;

% Normalized 2D points
normalizedpoints_2D =  T * points_2D ;

% Same for 3D

U_temp =  [1 0 0 -mean(points_3D(1,:)) ;
           0 1 0 -mean(points_3D(2,:));
           0 0 1 -mean(points_3D(3,:));
           0 0 0 1 ] ;

total_sum = 0 ;
XYZ_center = U_temp*points_3D ;
for i = 1:6
    total_sum = total_sum + norm(XYZ_center(1:3,i));
end
lamda_2D = 6*sqrt(3)/total_sum  ;  

% Transformation matrix U
U = lamda_2D * U_temp ;
U(4,4) = 1 ;

% Normalized 3D points
normalizedpoints_3D = U * points_3D;
%% Estimate the normalized projection matrix using DLT method and denormalize it.

% Create matrix M
M = [] ;
for i = 1:6
    M((i-1)*2+1,:) = [ normalizedpoints_2D(3,i)*normalizedpoints_3D(:,i)' zeros(1,4) -normalizedpoints_2D(1,i)*normalizedpoints_3D(:,i)' ] ;
    M((i-1)*2+2,:) = [zeros(1,4) -normalizedpoints_2D(3,i)*normalizedpoints_3D(:,i)' normalizedpoints_2D(2,i)*normalizedpoints_3D(:,i)' ] ;
end

% SVD and take eigenvector with smallest eigenvalue
[~,~,V1] = svd(M);

temp1 = V1(:,end);

P_normalized = [temp1(1:4)'; temp1(5:8)' ;temp1(9:12)' ];

% Denormalize camera matrix
P = (T\P_normalized)*U ;
%% Decompose camera matrix P into K, R and t using rq decomposition

[q,r] = qr(inv(P(1:3,1:3)));
temp2 = inv(r);
D = temp2(end,end) ;

% K matrix
K = (1/D)*temp2;

% R matrix
R = D*inv(q);

% C matrix
[~,~,V2] = svd(P);
C = V2(1:3,end)/V2(end,end) ;

% t matrix
t = - R*C ; 
%% Compute reprojection error

% Projected 2D points using P matrix 
projectedpoints_2D = P*points_3D ; 

% Scale for homogenous coordinate
for i = 1:n 
    scale = projectedpoints_2D(3,i) ;
    projectedpoints_2D(:,i) = 1/scale * projectedpoints_2D(:,i);
end

error_matrix = (projectedpoints_2D - points_2D) ;
error_matrix2= error_matrix.^2;
error = sum(error_matrix2(:));

Reprojection_error = sqrt(error)/n;
%% Visualization

% Show reference 2D points and estimated 2D points
img = imread(path);
figure
imshow(img); 
hold on ;

for i = 1:(size(points_2D,2))
    plot(points_2D(1,i),points_2D(2,i),'Marker','.','MarkerEdgeColor','r','MarkerSize',10,'linewidth',2); 
    hold on;
end

for i = 1:(size(projectedpoints_2D,2))
    plot(projectedpoints_2D(1,i),projectedpoints_2D(2,i),'Marker','.','MarkerEdgeColor','y','MarkerSize',10,'linewidth',2); 
    hold on;
end
hold off;

img = imread(path);
figure
imshow(img); 
hold on;


% Show checkboard corners
% For Left pannel
for i = 0:8
    for j = 0:8
        X = i*2 ; Y = 0 ; Z= j*2; 
        point = [X Y Z 1]';
        point = P*point;
        scale = point(3) ;
        point = (1/scale) * point;
        points = (point(1:2))';
        plot(points(1), points(2),'Marker','.','MarkerEdgeColor','y','MarkerSize',10,'linewidth', 1); 
        hold on;
    end
end
% For Right pannel
for i = 0:8
    for j = 0:8
        Y = i*2 ; X = 0 ; Z= j*2; 
        point = [X Y Z 1]';
        point = P*point;
        scale = point(3);
        point = 1/scale * point;
        points = (point(1:2))';
        plot(points(1), points(2),'Marker','.','MarkerEdgeColor','g','MarkerSize',10,'linewidth', 1); 
        hold on;
    end  
end
hold off;