currFrameIdx = 1;
D ='C:/Users/Amir/Desktop/ORB-SLAM/image_0/';
S = dir(fullfile(D,'*.png'));
N = natsortfiles({S.name});
F = cellfun(@(n)fullfile(D,n),N,'uni',0);
imds = imageDatastore(F);
bag = bagOfFeatures(imds,'CustomExtractor', @helperExtractorFunction);
k=1;
I1 = readimage(imds, currFrameIdx);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set random seed for reproducibility
rng(0);
% Create a cameraIntrinsics object to store the camera intrinsic parameters.
% The intrinsics for the dataset can be found at the following page:
% https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
% Note that the images in the dataset are already undistorted, hence there
% is no need to specify the distortion coefficients.
focalLength    = [0.4, 0.53];    % in units of pixels
principalPoint = [0.5, 0.5];   % in units of pixels
imageSize      = size(I1,[1 2]);  % in units of pixels
% intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example intrinsic matrix K
intrinsicMatrix = [0.4, 0, 0.5; 0, 0.53, 0.5; 0, 0, 1];

% Create camera parameters object
cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix);
currFrameIdx = currFrameIdx + 1;
% Map initialization loop
% Map initialization loop

firstI = I1; % Preserve the first frame 
[preFeatures, prePoints] = helperExtractorFunction(I1);
isMapInitialized  = false;

disp(preFeatures);
disp(size(prePoints));

while ~isMapInitialized 
    currI = readimage(imds, currFrameIdx);
    [currFeatures, currPoints] = helperExtractorFunction(currI); 
    len = min(length(preFeatures.Features),length(currFeatures.Features));
    currFrameIdx = currFrameIdx + 1;
    % Find putative feature matches
    indexPairs = matchFeatures(preFeatures.Features(1:len,:), currFeatures.Features(1:len,:),'Unique',true,'MaxRatio',0.9,'MatchThreshold',40);
    preMatchedPoints  = prePoints(indexPairs(:,1),:);
    currMatchedPoints = currPoints(indexPairs(:,2),:);
    % If not enough matches are found, check the next frame
    minMatches = 100;
    if size(indexPairs, 1) < minMatches
        continue
    end
    preMatchedPoints  = prePoints(indexPairs(:,1),:);
    currMatchedPoints = currPoints(indexPairs(:,2),:);
    % Compute homography and evaluate reconstruction
    [tformH, scoreH, inliersIdxH] = helperComputeHomography(preMatchedPoints, currMatchedPoints);
    % Compute fundamental matrix and evaluate reconstruction
    [tformF, scoreF, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints);
    % Select the model based on a heuristic
    ratio = scoreH/(scoreH + scoreF);
    ratioThreshold = 0.45;
    if ratio > ratioThreshold
        inlierTformIdx = inliersIdxH;
        tform          = tformH;
    else
        inlierTformIdx = inliersIdxF;
        tform          = tformF;
    end

    % Computes the camera location up to scale. Use half of the 
    % points to reduce computation
    inlierPrePoints  = preMatchedPoints(inlierTformIdx);
    inlierCurrPoints = currMatchedPoints(inlierTformIdx);
    [relPose, validFraction] = estrelpose(tform, intrinsics, ...
        inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));

    % If not enough inliers are found, move to the next frame
    if validFraction < 0.9 || numel(relPose)==3
        continue
    end

    % Triangulate two views to obtain 3-D map points
    minParallax = 1; % In degrees
    [isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
        rigidtform3d, relPose, inlierPrePoints, inlierCurrPoints, intrinsics, minParallax);

    if ~isValid
        continue
    end

    % Get the original index of features in the two key frames
    indexPairs = indexPairs(inlierTformIdx(inlierTriangulationIdx),:);

    isMapInitialized = true;

    disp(['Map initialized with frame 1 and frame ', num2str(currFrameIdx-1)])
end % End of map initialization loop

if isMapInitialized
    % Close the previous figure    % Show matched features
    hfeature = showMatchedFeatures(firstI, I2, prePoints(indexPairs(:,1)),currPoints(indexPairs(:, 2)), 'Montage');
else
    error('Unable to initialize map.')
end

% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% Create an empty helperMapPointSet object to store 3D map points
mapPointSet   = helperMapPointSet;

% Add the first key frame. Place the camera associated with the first 
% key frame at the origin, oriented along the Z-axis
preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigid3d, 'Points', prePoints,...
    'Features', preFeatures);
% Add the second key frame
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, 'Points', currPoints,...
    'Features', currFeatures);

% Add connection between the first and the second key frame
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, 'Matches', indexPairs);

% Add 3-D map points
[mapPointSet, newPointIdx] = addMapPoint(mapPointSet, xyzWorldPoints);

% Add observations of the map points
preLocations   = prePoints.Location;
currLocations  = currPoints.Location;
preScales      = prePoints.Scale;
currScales     = currPoints.Scale;

% Add image points corresponding to the map points in the first key frame
mapPointSet   = addObservation(mapPointSet, newPointIdx, preViewId, indexPairs(:,1), ....
    preLocations(indexPairs(:,1),:), preScales(indexPairs(:,1)));

% Add image points corresponding to the map points in the second key frame
mapPointSet   = addObservation(mapPointSet, newPointIdx, currViewId, indexPairs(:,2), ...
    currLocations(indexPairs(:,2),:), currScales(indexPairs(:,2)));

% Run full bundle adjustment on the first two key frames
tracks       = findTracks(vSetKeyFrames);
cameraPoses  = poses(vSetKeyFrames);

[refinedPoints, refinedAbsPoses] = bundleAdjustment(xyzWorldPoints, tracks, ...
    cameraPoses, intrinsics, 'FixedViewIDs', 1, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 50);

% Scale the map and the camera pose using the median depth of map points
medianDepth   = median(vecnorm(refinedPoints.'));
refinedPoints = refinedPoints / medianDepth;

refinedAbsPoses.AbsolutePose(currViewId).Translation = ...
    refinedAbsPoses.AbsolutePose(currViewId).Translation / medianDepth;
relPose.Translation = relPose.Translation/medianDepth;
% Update key frames with the refined poses
vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);
vSetKeyFrames = updateConnection(vSetKeyFrames, preViewId, currViewId, relPose);
% Update map points with the refined positions
mapPointSet = updateLocation(mapPointSet, refinedPoints);
% Update view direction and depth 
mapPointSet = updateViewAndRange(mapPointSet, vSetKeyFrames.Views, newPointIdx);
% Visualize matched features in the current frame
close(hfeature.Parent.Parent);
featurePlot = helperVisualizeMatchedFeatures(I2, currPoints(indexPairs(:,2)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize initial map points and camera trajectory
mapPlot     = helperVisualizeMotionAndStructure(vSetKeyFrames, mapPointSet);
% Show legend
showLegend(mapPlot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ViewId of the current key frame
currKeyFrameId    = currViewId;

% ViewId of the last key frame
lastKeyFrameId    = currViewId;

% ViewId of the reference key frame that has the most co-visible 
% map points with the current key frame
refKeyFrameId     = currViewId;

% Index of the last key frame in the input image sequence
lastKeyFrameIdx   = currFrameIdx - 1; 

% Indices of all the key frames in the input image sequence
addedFramesIdx    = [1; lastKeyFrameIdx];

isLoopClosed      = false;
while true
    currI = readimage(imds, currFrameIdx);
    
    [currFeatures, currPoints] =  helperExtractorFunction(currI);
    % Track the last key frame
    % mapPointsIdx:   Indices of the map points observed in the current frame
    % featureIdx:     Indices of the corresponding feature points in the 
    % current frameaddImages
    lastmapPointset = mapPointSet;
    lastvSetKeyFrames = vSetKeyFrames;
    [currPose, mapPointsIdx, featureIdx] = helperTrackLastKeyFrame(mapPointSet, ...
        vSetKeyFrames.Views, currFeatures, currPoints, lastKeyFrameId, intrinsics);

    [refKeyFrameId, localKeyFrameIds, currPose, mapPointsIdx, featureIdx] = ...
        helperTrackLocalMap(mapPointSet, vSetKeyFrames, mapPointsIdx, ...
        featureIdx, currPose, currFeatures, currPoints, intrinsics);
   
    %vSetKeyFrames = addView(vSetKeyFrames, lastKeyFrameIdx, rigid3d, 'Points', prePoints,...
    %'Features', preFeatures);

    isKeyFrame = helperIsKeyFrame(mapPointSet, refKeyFrameId, lastKeyFrameIdx, ...
    currFrameIdx, mapPointsIdx);
    
    % Visualize matched features
    updatePlot(featurePlot, currI, currPoints(featureIdx));
    
    if ~isKeyFrame 
     currFrameIdx = currFrameIdx + 1;
     continue
    end
   
    % Update current key frame ID
    currKeyFrameId  = currKeyFrameId + 1;
    % Add the new key frame 
    
    [mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(mapPointSet, vSetKeyFrames, ...
        currPose, currFeatures, currPoints, mapPointsIdx, featureIdx, localKeyFrameIds);
    
    % Update view direction and depth
    mapPointSet = updateViewAndRange(mapPointSet, vSetKeyFrames.Views, mapPointsIdx);
    
    % Remove outlier map points that are observed in fewer than 3 key frames
    mapPointSet = helperCullRecentMapPoints(mapPointSet, vSetKeyFrames, newPointIdx);
    
    % Create new map points by triangulation
    [mapPointSet, vSetKeyFrames, newPointIdx] = helperCreateNewMapPoints(mapPointSet,vSetKeyFrames,currKeyFrameId, intrinsics);

    % Local bundle adjustment
    [mapPointSet, vSetKeyFrames] = helperLocalBundleAdjustment(mapPointSet, vSetKeyFrames, ...
        currKeyFrameId, intrinsics); 
    
    % Visualize 3D world points and camera trajectory
    updatePlot(mapPlot, vSetKeyFrames, mapPointSet);
     % Initialize the loop closure database
    if currKeyFrameId == 3
        % Load the bag of features data created offline
        bofData         = bag;
        loopDatabase    = invertedImageIndex(bofData);
        loopCandidates  = [1; 2];
        
    % Check loop closure after some key frames have been created    
    elseif currKeyFrameId > 3
        
        % Detect possible loop closure key frame candidates
        [isDetected, validLoopCandidates] = helperCheckLoopClosure(vSetKeyFrames, currKeyFrameId, ...
            loopDatabase, currI, loopCandidates);
        
        if isDetected 
            % Add loop closure connections% Add loop closure connections
            if(isempty(mapPointSet.Locations))
                    currFrameIdx  = currFrameIdx + 1;
                    continue;
            end
            [isLoopClosed, mapPointSet, vSetKeyFrames] = helperAddLoopConnections(...
                mapPointSet, vSetKeyFrames, validLoopCandidates, ...
                currKeyFrameId, currFeatures, currPoints, intrinsics);
        end
    end
    
    % If no loop closure is detected, add the image into the database

    currds = imageDatastore(imds.Files{currFrameIdx});
    addImages(loopDatabase, currds, 'Verbose', false);
    loopCandidates= [loopCandidates; currKeyFrameId]; %#ok<AGROW>
    % Update IDs and indices
    lastKeyFrameId  = currKeyFrameId;
    lastKeyFrameIdx = currFrameIdx;
    currFrameIdx  = currFrameIdx + 1;
    addedFramesIdx  = [addedFramesIdx; currFrameIdx]; %#ok<AGROW>
    lastmapPointsIdx = mapPointsIdx;
    lastpose     = currPose;
    lastfeatureIdx=  featureIdx;
end

minNumMatches = 20;
vSetKeyFramesOptim = optimizePoses(vSetKeyFrames, minNumMatches, 'Tolerance', 1e-16, 'Verbose', true);
% Load ground truth 
gTruthData = load('orbslamGroundTruth.mat');
gTruth     = gTruthData.gTruth;

% Plot the actual camera trajectory 
plotActualTrajectory(mapPlot, gTruth(addedFramesIdx), optimizedPoses);

% Show legend
showLegend(mapPlot);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

