clear
close all
clc  
%%
% Use a fileDatastore object to manage large collections of custom format files
datafolder = 'data';
ds = fileDatastore(datafolder, 'ReadFcn', @pcread, 'FileExtensions', '.pcd'); 
%% 
% Matlab's implemantation 
gridStep = .2;

col1 = [208,231,32]/ 255;  col2 = [233,161,25]/ 255; 

% Create a figure and set its size to 1024x748 pixels
hfig =  figure('Position', [300, 200, 600, 750]); 
hax =   axes('Parent', hfig);

% Create a streaming point cloud display object
lidarPlayer = pcplayer([-100 200], [-100 100], [-5 15], ...
    'BackgroundColor', 'k', 'MarkerSize', 1, 'Parent', hax);

% Camera Settings
campos(hax, [0,0,30])
camtarget(hax, [0,0,0])
camup(hax, [0,1,0])
camproj(hax, 'orthographic')
camzoom(hax, 3)

axis(hax, 'off')

reset(ds)

% Load the first point cloud
fixed = read(ds); 

fixed.Color = ones(fixed.Count,1) .* col1;

% Downsample the fixed point cloud
fixedDownsampled = pcdownsample(fixed, 'gridAverage', gridStep); 

% Compute the normals for the fixed point cloud
fixedDownsampled.Normal = pcnormals(fixedDownsampled, 20);

% Initialize the map with the first point cloud
map = fixed;
% map = pcdownsample(fixed, 'gridAverage', .1);

% Initialize the agent's pose
tform = rigidtform3d;

% Visualize the initialized Map
view(lidarPlayer, pctransform(map, rigidtform3d([0,0,0],zeros(1,3)))); 

% Read a new point cloud
moving = read(ds); moving.Color = ones(moving.Count,1) .* col2;
 
tic
% Dowsample the moving point cloud
movingDownsampled = pcdownsample(moving, 'gridAverage', gridStep); 

% Compute the normals for the moving point cloud
movingDownsampled.Normal = pcnormals(movingDownsampled, 20);

% Estimate the transformation 
tform1 = pcregistericp(movingDownsampled, fixedDownsampled, ...
    'Metric', 'PlaneToPlane', 'InitialTransform', tform);
toc;
    
% Transform the moving point cloud 
movingReg = pctransform(moving, tform1);

% Merge the point cloud after registration 
map = pccat([map, movingReg]);
% map = pcmerge(map, movingReg, .1); 
    
% Visualize point cloud
view(lidarPlayer, pctransform(map, rigidtform3d([0,0,0],zeros(1,3))));
%%
% My implemantation 
gridStep = .1;

col1 = [208,231,32]/ 255;  col2 = [233,161,25]/ 255; 

% Create a figure and set its size to 1024x748 pixels
hfig =  figure('Position', [300, 200, 600, 750]); 
hax =   axes('Parent', hfig);

% Create a streaming point cloud display object
lidarPlayer = pcplayer([-100 200], [-100 100], [-5 15], ...
    'BackgroundColor', 'k', 'MarkerSize', 1, 'Parent', hax);

% Camera Settings
campos(hax, [0,0,30])
camtarget(hax, [0,0,0])
camup(hax, [0,1,0])
camproj(hax, 'orthographic')
camzoom(hax, 3)

axis(hax, 'off')

reset(ds)
total_toc = 0; 

% Load the first point cloud
fixed = read(ds); 

fixed.Color = ones(fixed.Count,1) .* col1;

% Downsample the fixed point cloud
fixedDownsampled = pcdownsample(fixed, 'gridAverage', gridStep); 

% Build KDTree for the fixed point cloud 
fixedKDTree = KDTreeSearcher(fixedDownsampled.Location);

% Build covariance matrices for the moving point cloud 
fixedCovariances = cov.get_covariance_matrices(fixedDownsampled.Location, fixedKDTree);

% Initialize the map with the first point cloud
map = fixed;
% map = pcdownsample(fixed, 'gridAverage', .1);

% Initialize the agent's pose
tform = rigidtform3d;

% Visualize the initialized Map
view(lidarPlayer, pctransform(map, rigidtform3d([0,0,0],zeros(1,3)))); 

% Read a new point cloud
moving = read(ds); moving.Color = ones(moving.Count,1) .* col2;
 
tic
% Dowsample the moving point cloud
movingDownsampled = pcdownsample(moving, 'gridAverage', gridStep); 

% Build KDTree for the moving point cloud 
movingKDTree = KDTreeSearcher(movingDownsampled.Location);

% Build covariance matrices for the moving point cloud 
movingCovariances = cov.get_covariance_matrices(movingDownsampled.Location, movingKDTree);
    
% Estimate the transformation 
tform2 = gicp(movingDownsampled.Location, fixedDownsampled.Location, ...
    movingCovariances, fixedCovariances, fixedKDTree, ...
    'InitialTransform', tform);
toc;

% Transform the moving point cloud 
movingReg = pctransform(moving, tform2);

% Merge the point cloud after registration 
map = pccat([map, movingReg]);
% map = pcmerge(map, movingReg, .1); 
    
% Visualize point cloud
view(lidarPlayer, pctransform(map, rigidtform3d([0,0,0],zeros(1,3))));
%%

% close all
% 
% figure 
% 
% pcshow(map)

[errt,erra] = calculateError(tform1, tform2);

errt
erra

function [errt,erra] = calculateError(tform, tform_GT)

R_GT    = tform_GT.R;
R       = tform.R;

t_GT    = tform_GT.Translation;
t       = tform.Translation;

erra    = rad2deg(acos((trace(R*R_GT')-1)/2));
errt    = norm(t-t_GT);

end